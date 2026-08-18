#include "payload/simulation_payload_adapter.hpp"
#include <chrono>

namespace full_self_driving::payload
{

SimulationPayloadAdapter::SimulationPayloadAdapter(const std::string & adapter_id)
: adapter_id_(adapter_id)
{
}

void SimulationPayloadAdapter::set_fault_mode(FaultMode mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fault_mode_ = mode;
}

void SimulationPayloadAdapter::set_healthy(bool healthy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  healthy_ = healthy;
}

void SimulationPayloadAdapter::set_simulated_latency_us(uint64_t latency_us)
{
  std::lock_guard<std::mutex> lock(mutex_);
  simulated_latency_us_ = latency_us;
}

bool SimulationPayloadAdapter::execute_command(
  uint8_t commanded_state,
  const std::string & operation_id,
  full_self_driving::msg::PayloadStatus & out_status)
{
  std::lock_guard<std::mutex> lock(mutex_);

  last_operation_id_ = operation_id;
  commanded_state_ = commanded_state;

  if (!healthy_ || fault_mode_ == FAULT_HARDWARE_ERROR) {
    feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_FAULT;
    last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
    unknown_result_ = false;
    out_status = get_status();
    return false;
  }

  if (fault_mode_ == FAULT_TIMEOUT || fault_mode_ == FAULT_POWER_LOSS) {
    feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_UNKNOWN;
    last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_UNKNOWN;
    unknown_result_ = true;
    out_status = get_status();
    return false;
  }

  if (fault_mode_ == FAULT_CONTRADICTORY_FEEDBACK) {
    feedback_state_ = (commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_OPEN) ?
      full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED :
      full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
    last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_UNKNOWN;
    unknown_result_ = true;
    out_status = get_status();
    return false;
  }

  // Nominal execution
  switch (commanded_state) {
    case full_self_driving::msg::PayloadStatus::COMMAND_OPEN:
      feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
      cargo_loaded_ = false;
      secured_ = false;
      last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
      unknown_result_ = false;
      break;

    case full_self_driving::msg::PayloadStatus::COMMAND_SECURED:
      feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
      cargo_loaded_ = true;
      secured_ = true;
      last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
      unknown_result_ = false;
      break;

    case full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED:
      feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
      cargo_loaded_ = false;
      secured_ = false;
      successful_operation_count_++;
      last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
      unknown_result_ = false;
      break;

    default:
      feedback_state_ = full_self_driving::msg::PayloadStatus::FEEDBACK_UNKNOWN;
      last_operation_result_ = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
      unknown_result_ = false;
      break;
  }

  out_status = get_status();
  return (last_operation_result_ == full_self_driving::msg::PayloadStatus::RESULT_SUCCESS);
}

full_self_driving::msg::PayloadStatus SimulationPayloadAdapter::get_status() const
{
  full_self_driving::msg::PayloadStatus status;
  status.header.frame_id = "payload";
  status.adapter_id = adapter_id_;
  status.commanded_state = commanded_state_;
  status.feedback_state = feedback_state_;
  status.cargo_loaded = cargo_loaded_;
  status.secured = secured_;
  status.successful_operation_count = successful_operation_count_;
  status.has_last_operation_id = !last_operation_id_.empty();
  status.last_operation_id = last_operation_id_;
  status.last_operation_result = last_operation_result_;
  status.unknown_result = unknown_result_;
  status.feedback_latency_us = simulated_latency_us_;
  status.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  return status;
}

}  // namespace full_self_driving::payload
