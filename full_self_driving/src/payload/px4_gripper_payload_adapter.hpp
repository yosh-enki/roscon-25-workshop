#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include "payload/payload_adapter.hpp"
#include "full_self_driving/msg/payload_status.hpp"

namespace full_self_driving::payload
{

struct Px4GripperConfig
{
  std::string adapter_id{"px4_uorb_gripper_actuator"};
  uint8_t gripper_instance{1};
  uint64_t command_timeout_ms{1500};
  uint8_t target_system{1};
  uint8_t target_component{1};
};

class Px4GripperPayloadAdapter : public PayloadAdapter
{
public:
  using Config = Px4GripperConfig;

  explicit Px4GripperPayloadAdapter(rclcpp::Node & node);
  Px4GripperPayloadAdapter(rclcpp::Node & node, Px4GripperConfig config);

  ~Px4GripperPayloadAdapter() override = default;

  std::string get_adapter_id() const override { return config_.adapter_id; }
  bool is_healthy() const override;

  bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) override;

  full_self_driving::msg::PayloadStatus get_status() const override;

  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck::SharedPtr ack);

private:
  rclcpp::Node & node_;
  Px4GripperConfig config_;
  mutable std::mutex mutex_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

  bool healthy_{true};
  full_self_driving::msg::PayloadStatus current_status_;
  uint64_t last_command_timestamp_us_{0};
  bool pending_ack_{false};
};

}  // namespace full_self_driving::payload
