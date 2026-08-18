#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class Property11MissionSequenceTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_prop11_node");
    context_ = std::make_shared<domain::MissionContext>("ctx_prop11");
    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("sim_adapter_prop11");
    controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);
    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);

    persistence::StoragePaths paths{"/tmp/prop11/state", "/tmp/prop11/plan", "/tmp/prop11/ev", "/tmp/prop11/bk"};
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

    coordinator_->set_payload_controller(controller_);
    coordinator_->set_persistence_manager(persistence_);

    // Pre-arm / pre-lock context for nominal sortie
    context_->set_engineering_config(std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config()));
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> controller_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
};

// 1. Property 11.1: Complete End-to-End Sortie Nominal Sequence
TEST_F(Property11MissionSequenceTest, FullEndToEndSortieSequence)
{
  std::string err;

  // Phase 1: Startup & Waiting for mode
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);

  // Phase 2: First Takeoff
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TAKEOFF, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TAKEOFF);

  // Phase 3: Transit In
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TRANSIT_IN, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  // Phase 4: Direct Acquisition (or Search)
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::DIRECT, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);

  // Phase 5: Precision Landing
  EXPECT_TRUE(coordinator_->handle_direct_complete());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PRECISION_LAND);

  // Phase 6: Landed Verified & Payload Operation
  EXPECT_TRUE(coordinator_->handle_landing_verified());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PAYLOAD_OPERATION);

  // Phase 7: Payload Release Success -> Second Takeoff
  EXPECT_TRUE(coordinator_->handle_payload_complete(msg::PayloadStatus::RESULT_SUCCESS));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TAKEOFF_AFTER_DELIVERY);

  // Phase 8: Transit Out
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::TRANSIT_OUT, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);

  // Phase 9: Return Strategy
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::RETURN_STRATEGY, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_STRATEGY);

  // Phase 10: Return Landed
  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::RETURN_LANDED, &err));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_LANDED);

  // Verify transition trace contains full sequential history
  const auto & trace = coordinator_->get_transition_trace();
  EXPECT_GE(trace.size(), 9u);
}

// 2. Property 11.2: Payload Non-Success Aborts to Return Strategy
TEST_F(Property11MissionSequenceTest, PayloadNonSuccessAbortsToReturn)
{
  coordinator_->request_transition(flight::StrategyType::PAYLOAD_OPERATION);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PAYLOAD_OPERATION);

  // When payload reports UNKNOWN or FAILURE -> transition directly to RETURN_STRATEGY
  coordinator_->handle_payload_complete(msg::PayloadStatus::RESULT_UNKNOWN);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_STRATEGY);
}
