#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;

class Property20AuthorityTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_p20_node");
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_p20");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);

    executor_->set_takeover_callback([this](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
      coordinator_->handle_takeover(reason);
    });
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
};

// Property 20.1: Manual RC / QGC takeover immediately overrides companion autonomy
TEST_F(Property20AuthorityTest, Property20_ManualTakeoverOverridesAutonomy)
{
  executor_->onActivate();
  mode_->onActivate();
  EXPECT_TRUE(executor_->is_active());
  EXPECT_FALSE(coordinator_->is_takeover_active());

  // RC switch or QGC mode change triggers deactivation
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);

  EXPECT_FALSE(executor_->is_active());
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}

// Property 20.2: PX4 failsafe immediately overrides companion autonomy
TEST_F(Property20AuthorityTest, Property20_FailsafeOverridesAutonomy)
{
  executor_->onActivate();
  mode_->onActivate();

  // FMU failsafe triggers deactivation
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::FailsafeActivated);

  EXPECT_FALSE(executor_->is_active());
  EXPECT_TRUE(coordinator_->is_takeover_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}

// Property 20.3: Lower-priority command sources cannot override or suppress takeover
TEST_F(Property20AuthorityTest, Property20_LowerPriorityCannotSuppressTakeover)
{
  executor_->onActivate();
  mode_->onActivate();

  // Trigger takeover
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
  EXPECT_TRUE(coordinator_->is_takeover_active());

  // Attempt transitions from lower-priority autonomy/gateway logic
  std::string err;
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::TAKEOFF, &err));
  EXPECT_FALSE(err.empty());

  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::TRANSIT_IN, &err));
  EXPECT_FALSE(err.empty());

  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::SEARCH, &err));
  EXPECT_FALSE(err.empty());

  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::PRECISION_LAND, &err));
  EXPECT_FALSE(err.empty());

  // Strategy remains safely in HOLD
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
}

// Property 20.4: Emergency stop has absolute veto authority
TEST_F(Property20AuthorityTest, Property20_EmergencyStopHasAbsoluteAuthority)
{
  EXPECT_FALSE(coordinator_->is_emergency_stop_active());

  coordinator_->handle_emergency_stop();
  EXPECT_TRUE(coordinator_->is_emergency_stop_active());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::FAILSAFE);

  // All transition attempts are strictly rejected
  std::string err;
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::TAKEOFF, &err));
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::WAITING_FOR_MODE, &err));
  EXPECT_FALSE(coordinator_->request_transition(flight::StrategyType::HOLD, &err));
}
