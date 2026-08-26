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
  init_publishers_and_timer();
}

void Px4GripperPayloadAdapter::init_publishers_and_timer()
{
  // 1. PX4 uXRCE-DDS expects Best Effort QoS (SensorDataQoS) for actuator inputs
  vehicle_command_pub_ = node_.create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", rclcpp::SensorDataQoS());

  actuator_servos_pub_ = node_.create_publisher<px4_msgs::msg::ActuatorServos>(
    "/fmu/in/actuator_servos", rclcpp::SensorDataQoS());

  offboard_control_mode_pub_ = node_.create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", rclcpp::SensorDataQoS());

  vehicle_command_ack_sub_ = node_.create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
    [this](const px4_msgs::msg::VehicleCommandAck::SharedPtr ack) {
      handle_command_ack(ack);
    });

  // 2. Stream timer at 20Hz (50ms interval) to sustain actuator pulses
  stream_timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { on_stream_timer(); });

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

void Px4GripperPayloadAdapter::on_stream_timer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stream_ticks_remaining_ <= 0) {
    return;
  }
  stream_ticks_remaining_--;

  uint64_t now_us = static_cast<uint64_t>(
    node_.get_clock()->now().nanoseconds() / 1000);

  // 1. Publish OffboardControlMode (direct_actuator)
  if (offboard_control_mode_pub_) {
    px4_msgs::msg::OffboardControlMode mode_msg{};
    mode_msg.timestamp = now_us;
    mode_msg.direct_actuator = true;
    offboard_control_mode_pub_->publish(mode_msg);
  }

  // 2. Publish ActuatorServos (AUX 1 / Servo 1)
  if (actuator_servos_pub_) {
    px4_msgs::msg::ActuatorServos servo_msg{};
    servo_msg.timestamp = now_us;
    uint8_t ch = (config_.gripper_instance >= 1) ? (config_.gripper_instance - 1) : 0;
    if (ch < 8) {
      servo_msg.control[ch] = active_target_val_;
    }
    actuator_servos_pub_->publish(servo_msg);
  }
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

  uint64_t now_us = static_cast<uint64_t>(
    node_.get_clock()->now().nanoseconds() / 1000);
  last_command_timestamp_us_ = now_us;

  bool is_secure_cmd = (commanded_state == full_self_driving::msg::PayloadStatus::COMMAND_SECURED);
  float actuator_target_val = is_secure_cmd ? config_.lock_value : config_.release_value;

  // 1. Trigger 20Hz ActuatorServos Stream for 1.5 seconds (30 ticks * 50ms)
  active_target_val_ = actuator_target_val;
  stream_ticks_remaining_ = 30;
  
  // Publish immediate first frame
  if (offboard_control_mode_pub_) {
    px4_msgs::msg::OffboardControlMode mode_msg{};
    mode_msg.timestamp = now_us;
    mode_msg.direct_actuator = true;
    offboard_control_mode_pub_->publish(mode_msg);
  }
  if (actuator_servos_pub_) {
    px4_msgs::msg::ActuatorServos servo_msg{};
    servo_msg.timestamp = now_us;
    uint8_t ch = (config_.gripper_instance >= 1) ? (config_.gripper_instance - 1) : 0;
    if (ch < 8) {
      servo_msg.control[ch] = active_target_val_;
    }
    actuator_servos_pub_->publish(servo_msg);
  }

  // 2. Command actuation via VehicleCommand fallback
  px4_msgs::msg::VehicleCommand cmd{};
  cmd.timestamp = now_us;
  if (config_.command_type == 187) {
    cmd.command = 187;
    switch (config_.gripper_instance) {
      case 2: cmd.param2 = actuator_target_val; break;
      case 3: cmd.param3 = actuator_target_val; break;
      case 4: cmd.param4 = actuator_target_val; break;
      case 5: cmd.param5 = actuator_target_val; break;
      case 6: cmd.param6 = actuator_target_val; break;
      case 1:
      default:
        cmd.param1 = actuator_target_val;
        break;
    }
    cmd.param7 = 0.0f; // Actuator Set 1 (0-indexed)
  } else {
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER;
    cmd.param1 = static_cast<float>(config_.gripper_instance);
    cmd.param2 = is_secure_cmd ? 1.0f : 0.0f;
  }

  switch (commanded_state) {
    case full_self_driving::msg::PayloadStatus::COMMAND_OPEN:
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN;
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED:
      current_status_.feedback_state = full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED;
      current_status_.cargo_loaded = false;
      current_status_.secured = false;
      break;
    case full_self_driving::msg::PayloadStatus::COMMAND_SECURED:
    default:
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
  if (!ack || (ack->command != config_.command_type &&
               ack->command != 187 &&
               ack->command != px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER)) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  pending_ack_ = false;

  uint64_t now_us = static_cast<uint64_t>(
    node_.get_clock()->now().nanoseconds() / 1000);

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
