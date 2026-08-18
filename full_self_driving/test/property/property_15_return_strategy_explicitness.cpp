#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/strategies/return_strategy.hpp"
#include "flight/strategies/transit_out_strategy.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class Property15ReturnStrategyExplicitnessTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_prop15_node");
    context_ = std::make_shared<domain::MissionContext>("ctx_prop15");
    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);

    persistence::StoragePaths paths{"/tmp/prop15/state", "/tmp/prop15/plan", "/tmp/prop15/ev", "/tmp/prop15/bk"};
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
};

// 1. Property 15.1: Outbound Route is Explicit, NOT Inbound Reversal
TEST_F(Property15ReturnStrategyExplicitnessTest, OutboundRouteExplicitNotReversed)
{
  auto inbound_route = domain::Route::create_default_kmitl_transit_in_route();
  auto outbound_route = domain::Route::create_default_kmitl_transit_out_route();

  // Verify that outbound route has explicit altitude separation (15.0m vs 10.0m)
  EXPECT_DOUBLE_EQ(outbound_route.get_transit_altitude_above_home_m(), 15.0);
  EXPECT_DOUBLE_EQ(inbound_route.get_transit_altitude_above_home_m(), 10.0);

  // Verify that RoutePoints maintain explicit altitude differences
  EXPECT_DOUBLE_EQ(outbound_route[0].altitude_m, 15.0);
  EXPECT_DOUBLE_EQ(inbound_route[0].altitude_m, 10.0);

  // Custom distinct egress corridor test
  domain::Route distinct_egress_route;
  distinct_egress_route.add_waypoint(domain::RoutePoint(13.730800, 100.789000, 15.0));
  distinct_egress_route.add_waypoint(domain::RoutePoint(13.730200, 100.787000, 15.0));
  coordinator_->set_custom_transit_out_route(distinct_egress_route);

  coordinator_->request_transition(flight::StrategyType::TAKEOFF_AFTER_DELIVERY);
  coordinator_->request_transition(flight::StrategyType::TRANSIT_OUT);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);
}

// 2. Property 15.2: Return Strategy Mode is Explicitly Governed
TEST_F(Property15ReturnStrategyExplicitnessTest, ReturnStrategyModeGoverned)
{
  // Test RTH Mode
  flight::ReturnStrategy rth_strategy(
    *node_, nullptr, nullptr, nullptr, persistence_, context_,
    flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME, 15.0);
  EXPECT_EQ(rth_strategy.get_return_mode(), flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME);

  // Test Immediate Land Mode
  flight::ReturnStrategy land_strategy(
    *node_, nullptr, nullptr, nullptr, persistence_, context_,
    flight::ReturnStrategy::ReturnMode::LAND_IMMEDIATELY, 15.0);
  EXPECT_EQ(land_strategy.get_return_mode(), flight::ReturnStrategy::ReturnMode::LAND_IMMEDIATELY);

  // Test Hold Mode
  flight::ReturnStrategy hold_strategy(
    *node_, nullptr, nullptr, nullptr, persistence_, context_,
    flight::ReturnStrategy::ReturnMode::HOLD_AT_FINAL_WAYPOINT, 15.0);
  EXPECT_EQ(hold_strategy.get_return_mode(), flight::ReturnStrategy::ReturnMode::HOLD_AT_FINAL_WAYPOINT);
}

// 3. Property 15.3: Coordinator Sequenced Transition to Return Strategy
TEST_F(Property15ReturnStrategyExplicitnessTest, CoordinatorTransitionsExplicitly)
{
  coordinator_->request_transition(flight::StrategyType::TRANSIT_OUT);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);

  std::string err;
  bool ok = coordinator_->request_transition(flight::StrategyType::RETURN_STRATEGY, &err);
  EXPECT_TRUE(ok);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_STRATEGY);

  ok = coordinator_->request_transition(flight::StrategyType::RETURN_LANDED, &err);
  EXPECT_TRUE(ok);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_LANDED);
}
