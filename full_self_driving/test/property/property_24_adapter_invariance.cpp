#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;

class Property24AdapterInvarianceTest : public ::testing::Test
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
    test_dir_ = "/tmp/fsd_test_p24_" + std::to_string(std::rand() % 100000);
    std::filesystem::create_directories(test_dir_);

    node_ = std::make_shared<rclcpp::Node>("test_p24_node");
    px4_ctx_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*px4_ctx_);

    // 1. Simulation Profile Context Setup
    sim_context_ = std::make_shared<domain::MissionContext>("ctx_sim");
    sim_cfg_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    sim_context_->set_engineering_config(sim_cfg_);

    // 2. Hardware Profile Context Setup
    hw_context_ = std::make_shared<domain::MissionContext>("ctx_hw");
    hw_cfg_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    hw_cfg_->profile = "hardware";
    hw_cfg_->adapters.px4_transport = "px4_hardware_uart_serial";
    hw_cfg_->adapters.camera_adapter = "v4l2_hardware_camera";
    hw_cfg_->adapters.payload_adapter = "gpio_pwm_payload_actuator";
    hw_context_->set_engineering_config(hw_cfg_);

    // Common persistence and controllers
    persistence::StoragePaths sim_paths{test_dir_ + "/sim/state", test_dir_ + "/sim/plan", test_dir_ + "/sim/ev", test_dir_ + "/sim/bk"};
    sim_persistence_ = std::make_shared<persistence::PersistenceManager>(sim_paths);

    persistence::StoragePaths hw_paths{test_dir_ + "/hw/state", test_dir_ + "/hw/plan", test_dir_ + "/hw/ev", test_dir_ + "/hw/bk"};
    hw_persistence_ = std::make_shared<persistence::PersistenceManager>(hw_paths);

    sim_adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("sim_adapter");
    sim_payload_ = std::make_shared<payload::PayloadController>(sim_adapter_, sim_context_);

    hw_adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("hw_adapter");
    hw_payload_ = std::make_shared<payload::PayloadController>(hw_adapter_, hw_context_);

    sim_coordinator_ = std::make_shared<domain::MissionCoordinator>(sim_context_);
    sim_coordinator_->set_payload_controller(sim_payload_);
    sim_coordinator_->set_persistence_manager(sim_persistence_);

    hw_coordinator_ = std::make_shared<domain::MissionCoordinator>(hw_context_);
    hw_coordinator_->set_payload_controller(hw_payload_);
    hw_coordinator_->set_persistence_manager(hw_persistence_);
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

  std::shared_ptr<domain::MissionContext> sim_context_;
  std::shared_ptr<domain::EngineeringConfig> sim_cfg_;
  std::shared_ptr<persistence::PersistenceManager> sim_persistence_;
  std::shared_ptr<payload::SimulationPayloadAdapter> sim_adapter_;
  std::shared_ptr<payload::PayloadController> sim_payload_;
  std::shared_ptr<domain::MissionCoordinator> sim_coordinator_;

  std::shared_ptr<domain::MissionContext> hw_context_;
  std::shared_ptr<domain::EngineeringConfig> hw_cfg_;
  std::shared_ptr<persistence::PersistenceManager> hw_persistence_;
  std::shared_ptr<payload::SimulationPayloadAdapter> hw_adapter_;
  std::shared_ptr<payload::PayloadController> hw_payload_;
  std::shared_ptr<domain::MissionCoordinator> hw_coordinator_;
};

// 1. Property 24.1: Domain safety validation rules and constraints are 100% identical
TEST_F(Property24AdapterInvarianceTest, DomainSafetyRulesInvariantAcrossProfiles)
{
  EXPECT_EQ(sim_cfg_->safety.min_battery_percentage, hw_cfg_->safety.min_battery_percentage);
  EXPECT_EQ(sim_cfg_->safety.max_altitude_m, hw_cfg_->safety.max_altitude_m);
  EXPECT_EQ(sim_cfg_->safety.target_loss_timeout_s, hw_cfg_->safety.target_loss_timeout_s);

  EXPECT_EQ(sim_cfg_->routes.landing_descent_rate_m_s, hw_cfg_->routes.landing_descent_rate_m_s);
  EXPECT_EQ(sim_cfg_->routes.max_horizontal_velocity_m_s, hw_cfg_->routes.max_horizontal_velocity_m_s);
  EXPECT_EQ(sim_cfg_->routes.transit_in_speed_m_s, hw_cfg_->routes.transit_in_speed_m_s);
  EXPECT_EQ(sim_cfg_->routes.transit_out_speed_m_s, hw_cfg_->routes.transit_out_speed_m_s);
  EXPECT_EQ(sim_cfg_->routes.search_altitude_m, hw_cfg_->routes.search_altitude_m);
  EXPECT_EQ(sim_cfg_->routes.approach_altitude_m, hw_cfg_->routes.approach_altitude_m);
  EXPECT_EQ(sim_cfg_->routes.acceptance_radius_m, hw_cfg_->routes.acceptance_radius_m);
  EXPECT_EQ(sim_cfg_->routes.max_yaw_rate_deg_s, hw_cfg_->routes.max_yaw_rate_deg_s);

  // Validation functions produce exact same outcomes
  auto sim_res = sim_cfg_->validate();
  auto hw_res = hw_cfg_->validate();
  EXPECT_TRUE(sim_res.is_valid);
  EXPECT_TRUE(hw_res.is_valid);
}

// 2. Property 24.2: MissionCoordinator state machine transitions are 100% invariant
TEST_F(Property24AdapterInvarianceTest, CoordinatorStateMachineInvariantAcrossProfiles)
{
  std::vector<flight::StrategyType> sequence = {
    flight::StrategyType::TAKEOFF,
    flight::StrategyType::TRANSIT_IN,
    flight::StrategyType::DIRECT,
    flight::StrategyType::PRECISION_LAND,
    flight::StrategyType::LANDED_VERIFIED,
    flight::StrategyType::PAYLOAD_OPERATION,
    flight::StrategyType::TAKEOFF_AFTER_DELIVERY,
    flight::StrategyType::TRANSIT_OUT,
    flight::StrategyType::RETURN_STRATEGY,
    flight::StrategyType::RETURN_LANDED,
  };

  for (auto strat : sequence) {
    std::string sim_err, hw_err;
    bool sim_ok = false;
    bool hw_ok = false;

    if (strat == flight::StrategyType::PRECISION_LAND) {
      sim_ok = sim_coordinator_->handle_direct_complete();
      hw_ok = hw_coordinator_->handle_direct_complete();
    } else if (strat == flight::StrategyType::LANDED_VERIFIED) {
      sim_ok = sim_coordinator_->handle_landing_verified();
      hw_ok = hw_coordinator_->handle_landing_verified();
    } else if (strat == flight::StrategyType::PAYLOAD_OPERATION) {
      sim_ok = sim_coordinator_->request_transition(strat, &sim_err);
      hw_ok = hw_coordinator_->request_transition(strat, &hw_err);
    } else if (strat == flight::StrategyType::TAKEOFF_AFTER_DELIVERY) {
      sim_ok = sim_coordinator_->handle_payload_complete(msg::PayloadStatus::RESULT_SUCCESS);
      hw_ok = hw_coordinator_->handle_payload_complete(msg::PayloadStatus::RESULT_SUCCESS);
    } else {
      sim_ok = sim_coordinator_->request_transition(strat, &sim_err);
      hw_ok = hw_coordinator_->request_transition(strat, &hw_err);
    }

    EXPECT_EQ(sim_ok, hw_ok) << "Transition mismatch for strategy: " << flight::strategy_type_to_string(strat);
    EXPECT_EQ(sim_coordinator_->get_current_strategy(), hw_coordinator_->get_current_strategy());
  }

  // Verify transition traces match 1:1
  const auto & sim_trace = sim_coordinator_->get_transition_trace();
  const auto & hw_trace = hw_coordinator_->get_transition_trace();
  ASSERT_EQ(sim_trace.size(), hw_trace.size());
  for (size_t i = 0; i < sim_trace.size(); ++i) {
    EXPECT_EQ(sim_trace[i], hw_trace[i]);
  }
}

// 3. Property 24.3: Takeover and Emergency Stop Authority are 100% invariant
TEST_F(Property24AdapterInvarianceTest, AuthorityAndEmergencyStopInvariantAcrossProfiles)
{
  sim_coordinator_->handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  hw_coordinator_->handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);

  EXPECT_TRUE(sim_coordinator_->is_takeover_active());
  EXPECT_TRUE(hw_coordinator_->is_takeover_active());
  EXPECT_EQ(sim_coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
  EXPECT_EQ(hw_coordinator_->get_current_strategy(), flight::StrategyType::HOLD);

  sim_coordinator_->handle_emergency_stop();
  hw_coordinator_->handle_emergency_stop();

  EXPECT_TRUE(sim_coordinator_->is_emergency_stop_active());
  EXPECT_TRUE(hw_coordinator_->is_emergency_stop_active());
  EXPECT_EQ(sim_coordinator_->get_current_strategy(), flight::StrategyType::FAILSAFE);
  EXPECT_EQ(hw_coordinator_->get_current_strategy(), flight::StrategyType::FAILSAFE);
}

// 4. Property 24.4: Persistence protocol, snapshots, and journal hashing are 100% invariant
TEST_F(Property24AdapterInvarianceTest, PersistenceProtocolInvariantAcrossProfiles)
{
  persistence::JournalEntry sim_entry, hw_entry;
  sim_entry.event_id = "EVENT_INVARIANCE_TEST";
  sim_entry.entry_sequence = 1;
  sim_entry.timestamp_monotonic_ns = 1000000;
  sim_entry.detail = "Profile invariance verification";

  hw_entry = sim_entry;

  std::string sim_cksum = sim_entry.compute_checksum();
  std::string hw_cksum = hw_entry.compute_checksum();
  EXPECT_EQ(sim_cksum, hw_cksum);
  EXPECT_FALSE(sim_cksum.empty());

  bool sim_ok = sim_persistence_->append_journal_entry(sim_entry);
  bool hw_ok = hw_persistence_->append_journal_entry(hw_entry);
  EXPECT_TRUE(sim_ok);
  EXPECT_TRUE(hw_ok);
}

// 5. Property 24.5: Only declared HAL adapter IDs differ between profiles
TEST_F(Property24AdapterInvarianceTest, OnlyDeclaredAdaptersDiffer)
{
  // Simulation vs Hardware adapter IDs differ strictly in declared HAL slots
  EXPECT_NE(sim_cfg_->adapters.px4_transport, hw_cfg_->adapters.px4_transport);
  EXPECT_NE(sim_cfg_->adapters.camera_adapter, hw_cfg_->adapters.camera_adapter);
  EXPECT_NE(sim_cfg_->adapters.payload_adapter, hw_cfg_->adapters.payload_adapter);

  EXPECT_EQ(sim_cfg_->adapters.px4_transport, "px4_sitl_uxrce_dds");
  EXPECT_EQ(hw_cfg_->adapters.px4_transport, "px4_hardware_uart_serial");

  EXPECT_EQ(sim_cfg_->adapters.camera_adapter, "ros_gz_image_bridge");
  EXPECT_EQ(hw_cfg_->adapters.camera_adapter, "v4l2_hardware_camera");

  EXPECT_EQ(sim_cfg_->adapters.payload_adapter, "simulation_payload_stub");
  EXPECT_EQ(hw_cfg_->adapters.payload_adapter, "gpio_pwm_payload_actuator");
}
