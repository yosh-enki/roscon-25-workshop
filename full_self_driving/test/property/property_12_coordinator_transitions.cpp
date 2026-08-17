#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "domain/live_target_lock.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;

class Property12CoordinatorTransitionsTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_p12_node");
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_p12");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
};

// Property 12.1: Perception observations and lock changes emit data only and never trigger strategy changes directly
TEST_F(Property12CoordinatorTransitionsTest, Property12_PerceptionDoesNotChangeFlightStrategy)
{
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);

  // Generate varied target lock events
  std::vector<domain::LiveTargetLock> events;

  domain::LiveTargetLock lock1;
  lock1.lock_state = domain::LockState::CANDIDATE;
  lock1.identity = domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026");
  events.push_back(lock1);

  domain::LiveTargetLock lock2;
  lock2.lock_state = domain::LockState::QUALIFIED;
  lock2.identity = domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026");
  events.push_back(lock2);

  domain::LiveTargetLock lock3;
  lock3.lock_state = domain::LockState::LOST;
  lock3.identity = domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026");
  events.push_back(lock3);

  for (const auto & ev : events) {
    coordinator_->handle_target_lock_update(ev);
    // Crucial Property 12 invariant: Flight strategy MUST remain unchanged by perception events alone
    EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);
    EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::WAITING_FOR_MODE);
  }
}

// Property 12.2: Strategy transitions are strictly coordinator-owned
TEST_F(Property12CoordinatorTransitionsTest, Property12_CoordinatorOwnsTransitions)
{
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);

  // Coordinator transitions through sequential mission stages
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TAKEOFF));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TAKEOFF);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TRANSIT_IN));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::ACQUIRE_TARGET);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::PRECISION_LAND));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PRECISION_LAND);

  const auto & trace = coordinator_->get_transition_trace();
  EXPECT_GT(trace.size(), 4u);
}

// Property 12.3: Single mode/executor path is the only transition effector
TEST_F(Property12CoordinatorTransitionsTest, Property12_SingleModeExecutorPathAppliesTransitions)
{
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::WAITING_FOR_MODE));
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::WAITING_FOR_MODE);

  // When executor is active, mode updates setpoints via the owning path
  mode_->onActivate();
  executor_->onActivate();

  EXPECT_NO_THROW(mode_->updateSetpoint(0.05f));

  mode_->onDeactivate();
  executor_->onDeactivate(flight::FullSelfDrivingModeExecutor::DeactivateReason::Other);
}
