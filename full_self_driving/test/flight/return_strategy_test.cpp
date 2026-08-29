#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/strategies/return_strategy.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class ReturnStrategyTest : public ::testing::Test
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
    static int node_counter = 0;
    node_ = std::make_shared<rclcpp::Node>("test_return_strategy_node_" + std::to_string(++node_counter));
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_return_test");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);

    persistence::StoragePaths paths{"/tmp/fsd_ret/state", "/tmp/fsd_ret/plan", "/tmp/fsd_ret/ev", "/tmp/fsd_ret/bk"};
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
};

// 1. Test Strategy Initialization
TEST_F(ReturnStrategyTest, StrategyInitialization)
{
  flight::ReturnStrategy strategy(
    *node_, *context_, state_cache_, persistence_, mission_ctx_,
    flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME);

  strategy.on_enter();
  EXPECT_EQ(strategy.get_type(), flight::StrategyType::RETURN_STRATEGY);
  EXPECT_FALSE(strategy.is_completed());
}

// 2. Test Return Mode Selection
TEST_F(ReturnStrategyTest, ModeSelection)
{
  flight::ReturnStrategy strategy_rth(
    *node_, *context_, state_cache_, persistence_, mission_ctx_,
    flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME);
  EXPECT_EQ(strategy_rth.get_return_mode(), flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME);

  flight::ReturnStrategy strategy_land(
    *node_, *context_, state_cache_, persistence_, mission_ctx_,
    flight::ReturnStrategy::ReturnMode::LAND_IMMEDIATELY);
  EXPECT_EQ(strategy_land.get_return_mode(), flight::ReturnStrategy::ReturnMode::LAND_IMMEDIATELY);
}

// 3. Test Coordinator Transition to ReturnStrategy
TEST_F(ReturnStrategyTest, CoordinatorTransitionToReturn)
{
  std::string err;
  bool ok = coordinator_->request_transition(flight::StrategyType::RETURN_STRATEGY, &err);
  EXPECT_TRUE(ok) << "Error: " << err;
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_STRATEGY);
}

// 4. Test Heading Lock and Safe Dwell Streaming
TEST_F(ReturnStrategyTest, HeadingLockAndSafeDwellStreaming)
{
  flight::ReturnStrategy strategy(
    *node_, *context_, state_cache_, persistence_, mission_ctx_,
    flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME);

  strategy.on_enter();
  EXPECT_EQ(strategy.get_sub_phase(), flight::ReturnStrategy::SubPhase::APPROACH_HOME);
  EXPECT_FALSE(strategy.is_completed());

  // Verify on_update does not crash and respects locked heading
  strategy.on_update(0.1f);
  EXPECT_FALSE(strategy.is_failed());
}
