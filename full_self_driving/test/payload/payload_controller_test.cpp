#include <gtest/gtest.h>
#include <memory>
#include <chrono>

#include "domain/mission_context.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "full_self_driving/srv/prepare_payload.hpp"

using namespace full_self_driving;

class PayloadControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<domain::MissionContext>("test_ctx");
    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("sim_payload_01");
    controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);
  }

  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> controller_;
};

// 1. Test Preflight Open For Loading
TEST_F(PayloadControllerTest, PreflightOpenForLoading)
{
  msg::PayloadStatus status;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok = controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "req_open_1",
    rev,
    status,
    &error);

  EXPECT_TRUE(ok) << "Error: " << error;
  EXPECT_EQ(status.commanded_state, msg::PayloadStatus::COMMAND_OPEN);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_OPEN);
  EXPECT_FALSE(status.cargo_loaded);
  EXPECT_FALSE(status.secured);
  EXPECT_EQ(status.last_operation_result, msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_FALSE(controller_->is_ready_for_sortie());
}

// 2. Test Preflight Verify Secured & Prepare For Sortie
TEST_F(PayloadControllerTest, PreflightPrepareForSortie)
{
  msg::PayloadStatus status;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok = controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "req_prep_1",
    rev,
    status,
    &error);

  EXPECT_TRUE(ok) << "Error: " << error;
  EXPECT_EQ(status.commanded_state, msg::PayloadStatus::COMMAND_SECURED);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_SECURED);
  EXPECT_TRUE(status.cargo_loaded);
  EXPECT_TRUE(status.secured);
  EXPECT_EQ(status.last_operation_result, msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_TRUE(controller_->is_ready_for_sortie());
}

// 3. Test Disarmed Requirement for Preflight Preparation
TEST_F(PayloadControllerTest, RejectsPreparationWhenArmedOrLocked)
{
  context_->set_armed(true);

  msg::PayloadStatus status;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok = controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "req_arm_1",
    rev,
    status,
    &error);

  EXPECT_FALSE(ok);
  EXPECT_FALSE(error.empty());
}

// 4. Test Idempotency: Duplicate Operation ID Returns Prior Result
TEST_F(PayloadControllerTest, IdempotentOperationHandling)
{
  msg::PayloadStatus status1;
  msg::PayloadStatus status2;
  std::string error;
  uint64_t rev = context_->get_selection_revision();

  bool ok1 = controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "idempotent_req_1",
    rev,
    status1,
    &error);
  EXPECT_TRUE(ok1);

  bool ok2 = controller_->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING, // Different op with duplicate ID
    "idempotent_req_1",
    rev,
    status2,
    &error);
  EXPECT_TRUE(ok2);
  // Returns prior cached result for idempotent_req_1
  EXPECT_EQ(status2.commanded_state, status1.commanded_state);
  EXPECT_EQ(status2.feedback_state, status1.feedback_state);
}

// 5. Test Internal In-Flight Release Execution
TEST_F(PayloadControllerTest, InternalReleaseExecution)
{
  // First prepare
  msg::PayloadStatus status;
  controller_->prepare(
    srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE,
    "prep_sortie",
    context_->get_selection_revision(),
    status);
  EXPECT_TRUE(controller_->is_ready_for_sortie());

  // Execute internal release
  std::string error;
  uint8_t res = controller_->execute_internal_release("rel_op_01", status, &error);

  EXPECT_EQ(res, msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(status.commanded_state, msg::PayloadStatus::COMMAND_RELEASE_REQUESTED);
  EXPECT_EQ(status.feedback_state, msg::PayloadStatus::FEEDBACK_RELEASED);
  EXPECT_FALSE(status.cargo_loaded);
  EXPECT_FALSE(status.secured);
  EXPECT_EQ(status.successful_operation_count, 1u);
  EXPECT_EQ(status.last_operation_result, msg::PayloadStatus::RESULT_SUCCESS);
}

// 6. Test Fault Injection: Timeout Outcome marked UNKNOWN and never retried
TEST_F(PayloadControllerTest, FaultTimeoutEntersUnknownResult)
{
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_TIMEOUT);

  msg::PayloadStatus status;
  std::string error;
  uint8_t res = controller_->execute_internal_release("timeout_op_01", status, &error);

  EXPECT_EQ(res, msg::PayloadStatus::RESULT_UNKNOWN);
  EXPECT_TRUE(status.unknown_result);
  EXPECT_EQ(status.last_operation_result, msg::PayloadStatus::RESULT_UNKNOWN);
}

// 7. Test Fault Injection: Contradictory Feedback
TEST_F(PayloadControllerTest, FaultContradictoryFeedbackEntersUnknownResult)
{
  adapter_->set_fault_mode(payload::SimulationPayloadAdapter::FAULT_CONTRADICTORY_FEEDBACK);

  msg::PayloadStatus status;
  std::string error;
  uint8_t res = controller_->execute_internal_release("contra_op_01", status, &error);

  EXPECT_EQ(res, msg::PayloadStatus::RESULT_UNKNOWN);
  EXPECT_TRUE(status.unknown_result);
}
