#include "hardware_payload_adapter.hpp"
#include <chrono>

namespace full_self_driving::payload
{

HardwarePayloadAdapter::HardwarePayloadAdapter(
  std::string device_path,
  int pwm_pin,
  int pwm_frequency_hz,
  int feedback_sense_pin)
: device_path_(std::move(device_path)),
  pwm_pin_(pwm_pin),
  pwm_frequency_hz_(pwm_frequency_hz),
  feedback_sense_pin_(feedback_sense_pin),
  healthy_(false)
{
  current_status_.header.frame_id = "hardware_payload";
  current_status_.adapter_id = adapter_id_;
  current_status_.commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_UNKNOWN;
  current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_UNKNOWN;
  current_status_.cargo_loaded = false;
  current_status_.secured = false;
  current_status_.successful_operation_count = 0;
  current_status_.has_last_operation_id = false;
  current_status_.last_operation_id = "";
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_NONE;
  current_status_.unknown_result = false;
  current_status_.feedback_latency_us = 0;
  current_status_.updated_monotonic_ns = 0;
}

std::string HardwarePayloadAdapter::get_adapter_id() const
{
  return adapter_id_;
}

bool HardwarePayloadAdapter::is_healthy() const
{
  // Hardware bringup is deferred pending physical validation package
  return healthy_;
}

bool HardwarePayloadAdapter::execute_command(
  uint8_t commanded_state,
  const std::string & operation_id,
  full_self_driving::msg::PayloadStatus & out_status)
{
  (void)commanded_state;
  // Fail-closed rejection for deferred physical hardware
  current_status_.has_last_operation_id = !operation_id.empty();
  current_status_.last_operation_id = operation_id;
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
  current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_FAULT;
  current_status_.unknown_result = false;
  current_status_.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  out_status = current_status_;
  return false;
}

full_self_driving::msg::PayloadStatus HardwarePayloadAdapter::get_status() const
{
  return current_status_;
}

std::string HardwarePayloadAdapter::get_device_path() const
{
  return device_path_;
}

int HardwarePayloadAdapter::get_pwm_pin() const
{
  return pwm_pin_;
}

int HardwarePayloadAdapter::get_pwm_frequency_hz() const
{
  return pwm_frequency_hz_;
}

int HardwarePayloadAdapter::get_feedback_sense_pin() const
{
  return feedback_sense_pin_;
}

}  // namespace full_self_driving::payload
