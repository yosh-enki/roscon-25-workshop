#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "adapters/px4_state_cache.hpp"
#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "flight/strategies/payload_operation_strategy.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class ResourceFailureIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    test_dir_ = "/tmp/fsd_test_res_fail_" + std::to_string(std::rand() % 100000);
    std::filesystem::create_directories(test_dir_);

    node_ = std::make_shared<rclcpp::Node>("test_res_fail_node");
    px4_ctx_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*px4_ctx_);

    context_ = std::make_shared<domain::MissionContext>("ctx_res_fail");
    auto cfg = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    context_->set_engineering_config(cfg);

    plan_manager_ = std::make_shared<runtime::PlanManager>(test_dir_ + "/plans");
    registry_ = std::make_shared<registry::PadRegistry>();

    persistence::StoragePaths paths;
    paths.state_directory = test_dir_ + "/state";
    paths.plan_directory = test_dir_ + "/plans";
    paths.evidence_directory = test_dir_ + "/evidence";
    paths.backup_directory = test_dir_ + "/backup";
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("res_sim_adapter");
    payload_controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);

    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);
    coordinator_->set_plan_manager(plan_manager_);
    coordinator_->set_pad_registry(registry_);
    coordinator_->set_payload_controller(payload_controller_);
    coordinator_->set_persistence_manager(persistence_);

    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_, state_cache_);
    coordinator_->bind_executor(executor_, mode_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::string test_dir_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> px4_ctx_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> registry_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> payload_controller_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
};

// 1. Payload Hardware Error Adapter Failure Fails Closed
TEST_F(ResourceFailureIntegrationTest, PayloadHardwareErrorFailsClosed)
{
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_HARDWARE_ERROR);

  msg::PayloadStatus status;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok = payload_controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "req_fail_hw_1",
    rev,
    status,
    &error);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(status.last_operation_result, msg::PayloadStatus::RESULT_FAILURE);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_FAULT);
  EXPECT_FALSE(payload_controller_->is_ready_for_sortie());
}

// 2. In-Flight Payload Timeout Fault Produces Explicit Unknown Result & Safe Recovery
TEST_F(ResourceFailureIntegrationTest, InFlightPayloadTimeoutOutcomeIsHandledSafely)
{
  // Prepare payload in preflight
  msg::PayloadStatus prep_status;
  payload_controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "prep_sortie_ok",
    context_->get_selection_revision(),
    prep_status);
  EXPECT_TRUE(payload_controller_->is_ready_for_sortie());

  // Inject timeout fault during in-flight delivery
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_TIMEOUT);

  auto strategy = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, payload_controller_, persistence_, context_, "op_delivery_timeout");

  strategy->on_enter();
  strategy->on_update(0.1f);

  EXPECT_TRUE(strategy->is_completed());
  EXPECT_EQ(strategy->get_result(), msg::PayloadStatus::RESULT_UNKNOWN);

  auto post_status = payload_controller_->get_status();
  EXPECT_TRUE(post_status.unknown_result);
  EXPECT_EQ(post_status.last_operation_result, msg::PayloadStatus::RESULT_UNKNOWN);
}

// 3. Adapter Power Loss / Unhealthy State Prevents Operation
TEST_F(ResourceFailureIntegrationTest, UnhealthyAdapterRejectsPreparation)
{
  adapter_->set_healthy(false);
  EXPECT_FALSE(adapter_->is_healthy());

  msg::PayloadStatus status;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok = payload_controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "req_power_loss",
    rev,
    status,
    &error);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(payload_controller_->is_ready_for_sortie());
}

// 4. Emergency Stop Fail-Closed Safety Overrides Active Execution
TEST_F(ResourceFailureIntegrationTest, EmergencyStopFailsClosedImmediately)
{
  mode_->onActivate();
  executor_->onActivate();
  context_->set_armed(true);
  coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  // Trigger emergency stop
  coordinator_->handle_emergency_stop();

  EXPECT_TRUE(coordinator_->is_emergency_stop_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::FAILSAFE);

  // Further transitions are strictly blocked under emergency stop
  std::string err;
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::TRANSIT_OUT, &err));
}

// 5. Perception Observation Queue Drops Handled Deterministically
TEST_F(ResourceFailureIntegrationTest, PadRegistryQueueDropsHandledGracefully)
{
  // Insert 200 observation records into registry
  for (uint32_t i = 1; i <= 200; ++i) {
    msg::PadRecord rec;
    rec.identity.marker_id = (i % 10) + 1;
    rec.identity.dictionary = "DICT_4X4_50";
    rec.identity.target_namespace = "aavc2026";
    rec.map_id = "kmitl_airfield";
    rec.scenario_id = "default_scenario";
    rec.latitude_deg = 13.731328;
    rec.longitude_deg = 100.789909;
    rec.altitude_m = 2.0;
    rec.quality = 0.9f;
    rec.last_observed_monotonic_ns = 100000000ULL + i;
    registry_->insert_record_for_test(rec);
  }

  // Verify registry remains valid and queries resolve accurately
  domain::TargetIdentity tid(1, "DICT_4X4_50", "aavc2026");
  auto match = registry_->lookup(tid, "kmitl_airfield", "default_scenario");
  EXPECT_TRUE(match.has_value());
  EXPECT_EQ(match->identity.marker_id, 1u);
}
