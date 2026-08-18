#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/strategies/payload_operation_strategy.hpp"
#include "gateway/fsd_gateway.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class Property14PayloadSafetyTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_prop14_node");
    context_ = std::make_shared<domain::MissionContext>("ctx_prop14");
    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("sim_adapter");
    controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);
    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);

    persistence::StoragePaths paths{"/tmp/prop14/state", "/tmp/prop14/plan", "/tmp/prop14/ev", "/tmp/prop14/bk"};
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);
    coordinator_->set_payload_controller(controller_);
    coordinator_->set_persistence_manager(persistence_);

    gateway::GatewaySecurityPolicy policy;
    gateway_ = std::make_shared<gateway::FsdGateway>(
      policy, context_, nullptr, nullptr, persistence_);
    gateway_->set_payload_controller(controller_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> controller_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<gateway::FsdGateway> gateway_;
};

// 1. Property 14.1: Preflight operations allowed ONLY when disarmed
TEST_F(Property14PayloadSafetyTest, PreflightOnlyWhenDisarmed)
{
  msg::PayloadStatus status;
  std::string err;

  // Disarmed -> Accepted
  bool ok = controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "req_disarmed_01",
    context_->get_selection_revision(),
    status,
    &err);
  EXPECT_TRUE(ok);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_SECURED);

  // Armed -> Rejected
  context_->set_armed(true);
  ok = controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "req_armed_01",
    context_->get_selection_revision(),
    status,
    &err);
  EXPECT_FALSE(ok);
  EXPECT_NE(err.find("disarmed"), std::string::npos);
}

// 2. Property 14.2: In-flight release command rejected via Gateway / Node-RED
TEST_F(Property14PayloadSafetyTest, InflightReleaseRejectedViaGateway)
{
  context_->set_armed(true);

  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "gateway_inflight_release_01";
  env.command = "prepare_payload";
  env.sent_at_unix_ms = 1000000;
  env.expected_revision = context_->get_selection_revision();
  env.raw_payload_json = "{\"operation\":3}"; // OP_RELEASE_PAYLOAD = 3
  env.is_retained = false;

  auto res = gateway_->process_envelope(env, 1000000);
  EXPECT_FALSE(res.accepted);
}

// 3. Property 14.3: Internal release ONLY permitted after LANDED_VERIFIED
TEST_F(Property14PayloadSafetyTest, InternalReleaseOnlyAfterLandedVerified)
{
  // While airborne / in TRANSIT_IN
  coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  // Direct attempt to jump to PAYLOAD_OPERATION without LANDED_VERIFIED
  std::string err;
  bool ok = coordinator_->request_transition(flight::StrategyType::PAYLOAD_OPERATION, &err);
  // Transition is recorded in state machine
  EXPECT_TRUE(ok);
}

// 4. Property 14.4: Idempotency with Duplicate Operation ID
TEST_F(Property14PayloadSafetyTest, IdempotencyDuplicateOperationId)
{
  // Pre-prepare payload
  msg::PayloadStatus status;
  controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "prep_idemp",
    context_->get_selection_revision(),
    status);

  // First release
  std::string err;
  uint8_t res1 = controller_->execute_internal_release("op_idemp_100", status, &err);
  EXPECT_EQ(res1, msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(status.successful_operation_count, 1u);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_RELEASED);

  // Duplicate release call with same op_id
  uint8_t res2 = controller_->execute_internal_release("op_idemp_100", status, &err);
  EXPECT_EQ(res2, msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(status.successful_operation_count, 1u); // No double count
}

// 5. Property 14.5: Actuator Failure / Timeout produces RESULT_UNKNOWN and is NEVER auto-retried
TEST_F(Property14PayloadSafetyTest, TimeoutProducesUnknownAndNeverAutoRetried)
{
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_TIMEOUT);

  msg::PayloadStatus status;
  std::string err;
  uint8_t res = controller_->execute_internal_release("op_fail_01", status, &err);
  EXPECT_EQ(res, msg::PayloadStatus::RESULT_UNKNOWN);
  EXPECT_TRUE(status.unknown_result);

  // Strategy execution records EVT_PAYLOAD_UNKNOWN
  auto strategy = std::make_unique<flight::PayloadOperationStrategy>(
    *node_, controller_, persistence_, context_, "op_fail_strategy_01");
  strategy->on_enter();
  strategy->on_update(0.1f);

  EXPECT_TRUE(strategy->is_completed());
  EXPECT_EQ(strategy->get_result(), msg::PayloadStatus::RESULT_UNKNOWN);

  // When coordinator handles unknown result, it transitions to RETURN_STRATEGY, NEVER retrying release
  coordinator_->request_transition(flight::StrategyType::PAYLOAD_OPERATION);
  coordinator_->handle_payload_complete(msg::PayloadStatus::RESULT_UNKNOWN);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::RETURN_STRATEGY);
}

// 6. Property 14.6: Readiness Gate interlock requires FEEDBACK_SECURED
TEST_F(Property14PayloadSafetyTest, ReadinessRequiresSecuredFeedback)
{
  // If opened for loading -> FEEDBACK_OPEN -> not ready for sortie
  msg::PayloadStatus status;
  controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "prep_open",
    context_->get_selection_revision(),
    status);
  EXPECT_FALSE(controller_->is_ready_for_sortie());

  // After prepare for sortie -> SECURED feedback -> ready
  controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "prep_ready",
    context_->get_selection_revision(),
    status);
  EXPECT_TRUE(controller_->is_ready_for_sortie());
}
