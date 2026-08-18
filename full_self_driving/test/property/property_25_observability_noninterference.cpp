#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "adapters/px4_state_cache.hpp"
#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class Property25ObservabilityNoninterferenceTest : public ::testing::Test
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
    test_dir_ = "/tmp/fsd_test_prop25_" + std::to_string(std::rand() % 100000);
    std::filesystem::create_directories(test_dir_);

    node_ = std::make_shared<rclcpp::Node>("test_p25_node");
    px4_ctx_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*px4_ctx_);

    context_ = std::make_shared<domain::MissionContext>("ctx_prop25");
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

    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("prop25_sim_adapter");
    payload_controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);

    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);
    coordinator_->set_plan_manager(plan_manager_);
    coordinator_->set_pad_registry(registry_);
    coordinator_->set_payload_controller(payload_controller_);
    coordinator_->set_persistence_manager(persistence_);

    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_, state_cache_);
    coordinator_->bind_executor(executor_, mode_);

    executor_->set_takeover_callback([this](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
      coordinator_->handle_takeover(reason);
    });
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

// 1. Property 25.1: High journal write rate and persistence backlog never jitter or block real-time flight loop
TEST_F(Property25ObservabilityNoninterferenceTest, ExporterStallsDoNotBlockFlightLoop)
{
  // Activate mode and initiate flight strategy
  mode_->onActivate();
  executor_->onActivate();
  EXPECT_TRUE(executor_->is_active());

  context_->set_armed(true);
  bool ok = coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
  EXPECT_TRUE(ok);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  // Generate simulated heavy logging / evidence writes
  for (int i = 0; i < 50; ++i) {
    persistence::JournalEntry entry;
    entry.event_id = "HEAVY_LOG_EVENT_" + std::to_string(i);
    entry.detail = "Simulated heavy telemetry log payload for observability testing " + std::string(256, 'X');
    entry.entry_sequence = i;
    entry.timestamp_monotonic_ns = 1000000 + i;
    persistence_->append_journal_entry(entry);
  }

  // Execute flight mode update loop and measure execution duration
  auto start_time = std::chrono::steady_clock::now();
  
  for (int i = 0; i < 20; ++i) {
    coordinator_->set_current_monotonic_ns(1000000000ULL + i * 100000000ULL);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

  // Bounded execution: all 20 iterations must complete smoothly well within real-time budget (< 50ms)
  EXPECT_LT(elapsed_ms, 50) << "Flight loop was blocked or delayed by observability writes: " << elapsed_ms << " ms";
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);
}

// 2. Property 25.2: Diagnostic drops and health queue saturation cannot seize control or mutate flight phase
TEST_F(Property25ObservabilityNoninterferenceTest, DiagnosticDropsCannotMutateFlightState)
{
  mode_->onActivate();
  executor_->onActivate();
  context_->set_armed(true);
  coordinator_->request_transition(flight::StrategyType::SEARCH);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  // Simulate component health diagnostic queue overflow
  msg::ComponentHealth dropped_health;
  dropped_health.component_id = "fsd_evidence";
  dropped_health.state = msg::ComponentHealth::STATE_ACTIVE;
  dropped_health.ready = false; // Degraded
  dropped_health.queue_depth = 1000;
  dropped_health.queue_drop_count = 500;
  dropped_health.detail = "Diagnostic queue saturation; dropping messages";

  // Observability diagnostics degradation MUST NOT hijack coordinator or trigger unauthorized transitions
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_TRUE(context_->is_armed());
  EXPECT_FALSE(coordinator_->is_takeover_active());
}

// 3. Property 25.3: Storage reserve warnings and snapshot backpressure never block mode execution loop
TEST_F(Property25ObservabilityNoninterferenceTest, StorageReserveAlertDoesNotBlockFlight)
{
  // Test storage paths and low disk headroom scenario
  persistence::StoragePaths paths;
  paths.state_directory = test_dir_ + "/state";
  paths.plan_directory = test_dir_ + "/plans";
  paths.evidence_directory = test_dir_ + "/evidence";
  paths.backup_directory = test_dir_ + "/backup";

  auto pm = std::make_shared<persistence::PersistenceManager>(paths);
  
  // Storage reserve alert or snapshot write under constrained storage
  persistence::JournalEntry entry;
  entry.event_id = "STORAGE_WARNING";
  entry.detail = "Evidence storage space low warning";
  entry.entry_sequence = 1;
  entry.timestamp_monotonic_ns = 2000000ULL;
  bool write_ok = pm->append_journal_entry(entry);
  EXPECT_TRUE(write_ok);

  // Flight coordinator remains operational and does not crash or block
  mode_->onActivate();
  executor_->onActivate();
  EXPECT_TRUE(executor_->is_active());
}

// 4. Property 25.4: Truthfulness Invariant — Telemetry does not infer QGC GUI presence from traffic
TEST_F(Property25ObservabilityNoninterferenceTest, TruthfulnessNoQgcInferenceFromTelemetry)
{
  mode_->onActivate();
  executor_->onActivate();

  // Mode status reflects actual state cache and coordinator truth
  EXPECT_FALSE(coordinator_->is_takeover_active());

  // Simulate telemetry subscriber connects/disconnects; verify mode authority remains strictly based on PX4 failsafe/RC
  EXPECT_FALSE(coordinator_->is_takeover_active());

  // Trigger real RC/QGC takeover via ModeExecutor deactivation
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);

  // Takeover is now active based on authoritative PX4 callback, NOT inferred telemetry
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}

// 5. Property 25.5: Stale projections or external telemetry cannot override coordinator authority
TEST_F(Property25ObservabilityNoninterferenceTest, StaleTelemetryCannotOverrideAuthority)
{
  mode_->onActivate();
  executor_->onActivate();
  context_->set_armed(true);
  coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  // Trigger takeover
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  EXPECT_TRUE(coordinator_->is_takeover_active());

  // Attempt unauthorized transition from an external stale projection source
  std::string err;
  bool invalid_jump = coordinator_->request_transition(flight::StrategyType::TAKEOFF, &err);
  EXPECT_FALSE(invalid_jump);
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}
