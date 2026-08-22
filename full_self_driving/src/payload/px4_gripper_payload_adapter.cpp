#include "payload/px4_gripper_payload_adapter.hpp"

namespace full_self_driving::payload
{

Px4GripperPayloadAdapter::Px4GripperPayloadAdapter(rclcpp::Node & node)
: Px4GripperPayloadAdapter(node, Px4GripperConfig{})
{
}

Px4GripperPayloadAdapter::Px4GripperPayloadAdapter(
  rclcpp::Node & node,
  Px4GripperConfig config)
: node_(node), config_(std::move(config))
{
  vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", rclcpp::SystemDefaultsQoS());

  vehicle_command_ack_sub_ = node_.create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", rclcpp::SystemDefaultsQoS(),
    [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr ack) {
      handle_command_ack(ack);
    });

  current_status_.header.frame_id = "px4_gripper";
  current_status_.adapter_id = config_.adapter_id;
  current_status_.commanded_state = full_self_driving::msg::PayloadStatus::COMMAND_SECURED;
  current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
  current_status_.cargo_loaded = true;
  current_status_.secured = true;
  current_status_.successful_operation_count = 0;
  current_status_.has_last_operation_id = false;
  current_status_.last_operation_id = "";
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_NONE;
  current_status_.unknown_result = false;
  current_status_.feedback_latency_us = 0;
  current_status_.updated_monotonic_ns = 0;
}

bool Px4GripperPayloadAdapter::is_healthy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return healthy_;
}

bool Px4GripperPayloadAdapter::execute_command(
  uint8_t commanded_state,
  const std::string & operation_id,
  full_self_driving::msg::PayloadStatus & out_status)
{
  std::lock_guard<std::mutex> lock(mutex_);

  current_status_.has_last_operation_id = !operation_id.empty();
  current_status_.last_operation_id = operation_id;
  current_status_.commanded_state = commanded_state;

  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  last_command_timestamp_us_ = cmd.timestamp;

  cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER;
  cmd.param1 = static_cast<float>(config_.gripper_instance);

  switch (commanded_state) {
    case full_self_driving::msg::PayloadStatus::COMMAND_OPEN:
      cmd.param2 = 0.0f; // Release / Open
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED:
      cmd.param2 = 0.0f; // Release
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_SECURED:
    default:
      cmd.param2 = 1.0f; // Grab / Close
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
      current_status_.cargo_loaded = true;
      current_status_.secured = true;
      break;
  }

  cmd.target_system = config_.target_system;
  cmd.target_component = config_.target_component;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;

  if (vehicle_command_pub_) {
    vehicle_command_pub_->publish(cmd);
  }

  pending_ack_ = true;
  current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
  current_status_.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  out_status = current_status_;
  return true;
}

full_self_driving::msg::PayloadStatus Px4GripperPayloadAdapter::get_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_status_;
}

void Px4GripperPayloadAdapter::handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr ack)
{
  if (!ack || ack->command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pending_ack_ = false;

  uint64_t now_us = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  if (now_us >= last_command_timestamp_us_) {
    current_status_.feedback_latency_us = now_us - last_command_timestamp_us_;
  }

  if (ack->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
    current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_SUCCESS;
    if (current_status_.commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED) {
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
    } else if (current_status_.commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_OPEN) {
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
    } else if (current_status_.commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_SECURED) {
      current_status_.cargo_loaded = true;
      current_status_.secured = true;
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED;
    }
    current_status_.successful_operation_count++;
  } else {
    current_status_.last_operation_result = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
    current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_FAULT;
  }

  current_status_.updated_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

}  // namespace full_self_driving::payload
