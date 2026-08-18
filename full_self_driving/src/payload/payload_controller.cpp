#include "payload/payload_controller.hpp"

namespace full_self_driving::payload
{

PayloadController::PayloadController(
  std::shared_ptr<PayloadAdapter> adapter,
  std::shared_ptr<domain::MissionContext> context)
: adapter_(std::move(adapter)), context_(std::move(context))
{
}

void PayloadController::set_adapter(std::shared_ptr<PayloadAdapter> adapter)
{
  std::lock_guard<std::mutex> lock(mutex_);
  adapter_ = std::move(adapter);
}

std::shared_ptr<PayloadAdapter> PayloadController::get_adapter() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return adapter_;
}

void PayloadController::set_mission_context(std::shared_ptr<domain::MissionContext> context)
{
  std::lock_guard<std::mutex> lock(mutex_);
  context_ = std::move(context);
}

std::shared_ptr<domain::MissionContext> PayloadController::get_mission_context() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return context_;
}

bool PayloadController::prepare(
  uint8_t operation,
  const std::string & request_id,
  uint64_t expected_revision,
  full_self_driving::msg::PayloadStatus & out_status,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (context_) {
    if (context_->is_armed() || context_->is_locked()) {
      if (out_error) {
        *out_error = "Payload preparation requires disarmed and unlocked context";
      }
      return false;
    }

    if (expected_revision != 0 && expected_revision != context_->get_selection_revision()) {
      if (out_error) {
        *out_error = "Selection revision mismatch: expected " +
                     std::to_string(expected_revision) + " but current is " +
                     std::to_string(context_->get_selection_revision());
      }
      return false;
    }
  }

  // Check idempotency cache
  auto it = idempotency_records_.find(request_id);
  if (it != idempotency_records_.end()) {
    out_status = it->second;
    return (it->second.last_operation_result == full_self_driving::msg::PayloadStatus::RESULT_SUCCESS);
  }

  if (!adapter_) {
    if (out_error) {
      *out_error = "PayloadAdapter is not configured";
    }
    return false;
  }

  uint8_t commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_UNKNOWN;
  switch (operation) {
    case full_self_driving::srv::PreparePayload::Request::OP_OPEN_FOR_LOADING:
      commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_OPEN;
      break;
    case full_self_driving::srv::PreparePayload::Request::OP_VERIFY_SECURED:
    case full_self_driving::srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE:
      commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_SECURED;
      break;
    default:
      if (out_error) {
        *out_error = "Unknown or unsupported preparation operation: " + std::to_string(operation);
      }
      return false;
  }

  bool ok = adapter_->execute_command(commanded_state, request_id, out_status);
  idempotency_records_[request_id] = out_status;
  if (!ok && out_error && out_error->empty()) {
    *out_error = "PayloadAdapter command execution failed";
  }
  return ok;
}

uint8_t PayloadController::execute_internal_release(
  const std::string & operation_id,
  full_self_driving::msg::PayloadStatus & out_status,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Check idempotency cache
  auto it = idempotency_records_.find(operation_id);
  if (it != idempotency_records_.end()) {
    out_status = it->second;
    return it->second.last_operation_result;
  }

  if (!adapter_) {
    if (out_error) {
      *out_error = "PayloadAdapter is not configured";
    }
    out_status.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
    return full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
  }

  adapter_->execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED,
    operation_id,
    out_status);

  idempotency_records_[operation_id] = out_status;
  return out_status.last_operation_result;
}

full_self_driving::msg::PayloadStatus PayloadController::get_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (adapter_) {
    return adapter_->get_status();
  }
  full_self_driving::msg::PayloadStatus empty_status;
  return empty_status;
}

bool PayloadController::is_ready_for_sortie(std::string * out_reason) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!adapter_) {
    if (out_reason) *out_reason = "Payload adapter missing";
    return false;
  }

  if (!adapter_->is_healthy()) {
    if (out_reason) *out_reason = "Payload adapter unhealthy";
    return false;
  }

  auto status = adapter_->get_status();
  if (status.unknown_result) {
    if (out_reason) *out_reason = "Payload in unknown result state";
    return false;
  }

  if (!status.cargo_loaded || !status.secured ||
      status.feedback_state != full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED) {
    if (out_reason) *out_reason = "Payload cargo not loaded and secured";
    return false;
  }

  return true;
}

void PayloadController::reset_idempotency()
{
  std::lock_guard<std::mutex> lock(mutex_);
  idempotency_records_.clear();
}

}  // namespace full_self_driving::payload
