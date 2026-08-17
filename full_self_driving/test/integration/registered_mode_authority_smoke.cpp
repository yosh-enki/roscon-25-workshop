#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"

using namespace full_self_driving;

class RegisteredModeAuthoritySmokeTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_authority_smoke_node");
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_authority_smoke");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
};

TEST_F(RegisteredModeAuthoritySmokeTest, ModeNameAndRequirements)
{
  auto mode = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);

  // Verify requirements are set
  auto & reqs = mode->modeRequirements();
  EXPECT_TRUE(reqs.angular_velocity);
  EXPECT_TRUE(reqs.attitude);
  EXPECT_TRUE(reqs.local_position);
  EXPECT_TRUE(reqs.global_position);
  EXPECT_TRUE(reqs.home_position);

  // Initial strategy is WAITING_FOR_MODE
  EXPECT_EQ(mode->get_current_strategy_type(), flight::StrategyType::WAITING_FOR_MODE);
  EXPECT_EQ(mode->get_current_strategy_name(), "WAITING_FOR_MODE");
}

TEST_F(RegisteredModeAuthoritySmokeTest, ArmingCheckReporterIntegration)
{
  auto mode = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);

  bool is_ready = false;
  std::vector<std::string> missing_gates = {"Context is not committed", "PX4 transport not ready"};

  mode->set_readiness_check_callback([&](std::vector<std::string> & codes) -> bool {
    codes = missing_gates;
    return is_ready;
  });

  px4_msgs::msg::ArmingCheckReply reply;
  reply.can_arm_and_run = true;
  reply.num_events = 0;
  px4_ros2::HealthAndArmingCheckReporter reporter(reply);

  // When not ready, reporter should flag can_arm_and_run = false and add event
  mode->checkArmingAndRunConditions(reporter);
  EXPECT_FALSE(reply.can_arm_and_run);
  EXPECT_GT(reply.num_events, 0u);

  // When ready, reporter should preserve can_arm_and_run = true
  is_ready = true;
  missing_gates.clear();
  reply.can_arm_and_run = true;
  reply.num_events = 0;
  mode->checkArmingAndRunConditions(reporter);
  EXPECT_TRUE(reply.can_arm_and_run);
  EXPECT_EQ(reply.num_events, 0u);
}

TEST_F(RegisteredModeAuthoritySmokeTest, ActivationDeactivationCycles)
{
  auto mode = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
  auto executor = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode);

  bool mode_active = false;
  bool executor_active = false;

  mode->set_activation_callback([&](bool active) { mode_active = active; });
  executor->set_activation_callback([&](bool active) { executor_active = active; });

  EXPECT_FALSE(mode_active);
  EXPECT_FALSE(executor_active);

  // Simulate activation
  mode->onActivate();
  executor->onActivate();
  EXPECT_TRUE(mode_active);
  EXPECT_TRUE(executor_active);
  EXPECT_TRUE(executor->is_active());

  // Simulate deactivation
  executor->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  mode->onDeactivate();
  EXPECT_FALSE(mode_active);
  EXPECT_FALSE(executor_active);
  EXPECT_FALSE(executor->is_active());
  EXPECT_EQ(executor->last_deactivate_reason(), flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
}

TEST_F(RegisteredModeAuthoritySmokeTest, ManualTakeoverImmediatelyDeactivatesAndYields)
{
  auto mode = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
  auto executor = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode);
  coordinator_->bind_executor(executor, mode);

  bool takeover_notified = false;
  executor->set_takeover_callback([&](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
    takeover_notified = true;
    coordinator_->handle_takeover(reason);
  });

  // Activate
  executor->onActivate();
  mode->onActivate();
  EXPECT_FALSE(coordinator_->is_takeover_active());

  // Operator triggers RC / QGC takeover
  executor->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  EXPECT_TRUE(takeover_notified);
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);

  // Attempting normal autonomous transition during takeover must fail
  std::string err;
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::TAKEOFF, &err));
  EXPECT_FALSE(err.empty());
}

TEST_F(RegisteredModeAuthoritySmokeTest, FailsafeDeactivationTransitionsToHold)
{
  auto mode = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
  auto executor = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode);
  coordinator_->bind_executor(executor, mode);

  executor->set_takeover_callback([&](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
    coordinator_->handle_takeover(reason);
  });

  executor->onActivate();
  mode->onActivate();

  // FMU triggers failsafe
  executor->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::FailsafeActivated);
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}
