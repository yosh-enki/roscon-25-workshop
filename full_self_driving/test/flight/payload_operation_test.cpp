#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/strategies/payload_operation_strategy.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class PayloadOperationStrategyTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_payload_strategy_node");
    context_ = std::make_shared<domain::MissionContext>("ctx_test");
    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("sim_adapter");
    controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);

    persistence::StoragePaths paths{"/tmp/fsd_test/state", "/tmp/fsd_test/plan", "/tmp/fsd_test/ev", "/tmp/fsd_test/bk"};
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

    // Prepare payload in preflight
    msg::PayloadStatus status;
    controller_->prepare(
      srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
      "preflight_prep",
      context_->get_selection_revision(),
      status);

    context_->lock("mission_01", "sortie_01");
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> controller_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
};

// 1. Test Nominal Payload Release Strategy
TEST_F(PayloadOperationStrategyTest, NominalPayloadRelease)
{
  auto strategy = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, controller_, persistence_, context_, "op_rel_01");

  strategy->on_enter();
  strategy->on_update(0.1f);

  EXPECT_TRUE(strategy->is_completed());
  EXPECT_EQ(strategy->get_result(), msg::PayloadStatus::RESULT_SUCCESS);

  auto status = controller_->get_status();
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_RELEASED);
  EXPECT_FALSE(status.cargo_loaded);
  EXPECT_EQ(status.successful_operation_count, 1u);
}

// 2. Test Fault Timeout Outcome is UNKNOWN and Persisted
TEST_F(PayloadOperationStrategyTest, FaultTimeoutOutcomeIsUnknown)
{
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_TIMEOUT);

  auto strategy = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, controller_, persistence_, context_, "op_timeout_01");

  strategy->on_enter();
  strategy->on_update(0.1f);

  EXPECT_TRUE(strategy->is_completed());
  EXPECT_EQ(strategy->get_result(), msg::PayloadStatus::RESULT_UNKNOWN);

  auto status = controller_->get_status();
  EXPECT_TRUE(status.unknown_result);
}

// 3. Test Idempotency with Same Operation ID
TEST_F(PayloadOperationStrategyTest, IdempotentRelease)
{
  auto strategy1 = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, controller_, persistence_, context_, "op_dup_01");
  strategy1->on_enter();
  strategy1->on_update(0.1f);
  EXPECT_EQ(strategy1->get_result(), msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(controller_->get_status().successful_operation_count, 1u);

  // Run duplicate op ID
  auto strategy2 = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, controller_, persistence_, context_, "op_dup_01");
  strategy2->on_enter();
  strategy2->on_update(0.1f);
  EXPECT_EQ(strategy2->get_result(), msg::PayloadStatus::RESULT_SUCCESS);
  // Count should still be 1 (no double increment)
  EXPECT_EQ(controller_->get_status().successful_operation_count, 1u);
}
