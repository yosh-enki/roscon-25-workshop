#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include "payload/px4_gripper_payload_adapter.hpp"
#include "full_self_driving/msg/payload_status.hpp"

class Px4GripperPayloadAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("px4_gripper_adapter_test_node");
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

TEST_F(Px4GripperPayloadAdapterTest, PublishesVehicleCommandOnRelease)
{
  full_self_driving::payload::Px4GripperPayloadAdapter::Config cfg;
  cfg.adapter_id = "px4_uorb_gripper_actuator";
  cfg.gripper_instance = 1;
  cfg.command_type = 187; // MAV_CMD_DO_SET_ACTUATOR

  auto adapter = std::make_shared<full_self_driving::payload::Px4GripperPayloadAdapter>(*node_, cfg);
  EXPECT_EQ(adapter->get_adapter_id(), "px4_uorb_gripper_actuator");
  EXPECT_TRUE(adapter->is_healthy());

  bool command_received = false;
  uint32_t received_cmd = 0;
  float received_param1 = 0.0f;

  auto sub = node_->create_subscription<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", rclcpp::SensorDataQoS(),
    [&](const px4_msgs::msg::VehicleCommand::SharedPtr msg) {
      command_received = true;
      received_cmd = msg->command;
      received_param1 = msg->param1;
    });

  full_self_driving::msg::PayloadStatus status;
  bool ok = adapter->execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED,
    "op_test_001",
    status);

  EXPECT_TRUE(ok);
  EXPECT_EQ(status.last_operation_id, "op_test_001");
  EXPECT_EQ(status.commanded_state, full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED);

  // Spin to receive published message
  for (int i = 0; i < 5 && !command_received; ++i) {
    rclcpp::spin_some(node_);
  }

  EXPECT_TRUE(command_received);
  EXPECT_EQ(received_cmd, 187); // MAV_CMD_DO_SET_ACTUATOR
  EXPECT_FLOAT_EQ(received_param1, -1.0f); // -1.0 = Release
}

TEST_F(Px4GripperPayloadAdapterTest, ProcessesAckFeedback)
{
  full_self_driving::payload::Px4GripperPayloadAdapter adapter(*node_);

  full_self_driving::msg::PayloadStatus status;
  adapter.execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_RELEASE_REQUESTED,
    "op_test_002",
    status);

  // Simulate ACK from PX4 for command 187
  auto ack = std::make_shared<px4_msgs::msg::VehicleCommandAck>();
  ack->command = 187;
  ack->result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  ack->timestamp = node_->get_clock()->now().nanoseconds() / 1000;

  adapter.handle_command_ack(ack);

  auto updated = adapter.get_status();
  EXPECT_EQ(updated.last_operation_result, full_self_driving::msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(updated.feedback_state, full_self_driving::msg::PayloadStatus::FEEDBACK_RELEASED);
  EXPECT_FALSE(updated.cargo_loaded);
  EXPECT_FALSE(updated.secured);
}

TEST_F(Px4GripperPayloadAdapterTest, PreflightOpenMarksPayloadUnsecured)
{
  full_self_driving::payload::Px4GripperPayloadAdapter adapter(*node_);

  full_self_driving::msg::PayloadStatus status;
  bool ok = adapter.execute_command(
    full_self_driving::msg::PayloadStatus::COMMAND_OPEN,
    "op_test_open_001",
    status);

  EXPECT_TRUE(ok);
  EXPECT_EQ(status.last_operation_id, "op_test_open_001");
  EXPECT_EQ(status.commanded_state, full_self_driving::msg::PayloadStatus::COMMAND_OPEN);
  EXPECT_EQ(status.feedback_state, full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN);
  EXPECT_FALSE(status.cargo_loaded);
  EXPECT_FALSE(status.secured);

  // Simulate ACK from PX4 for command 187
  auto ack = std::make_shared<px4_msgs::msg::VehicleCommandAck>();
  ack->command = 187;
  ack->result = px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
  ack->timestamp = node_->get_clock()->now().nanoseconds() / 1000;

  adapter.handle_command_ack(ack);

  auto updated = adapter.get_status();
  EXPECT_EQ(updated.last_operation_result, full_self_driving::msg::PayloadStatus::RESULT_SUCCESS);
  EXPECT_EQ(updated.feedback_state, full_self_driving::msg::PayloadStatus::FEEDBACK_OPEN);
  EXPECT_FALSE(updated.cargo_loaded);
  EXPECT_FALSE(updated.secured);
}
