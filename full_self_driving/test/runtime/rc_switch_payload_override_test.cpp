#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

#include "runtime/flight_runtime_node.hpp"
#include "domain/mission_context.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"

using namespace full_self_driving;

class RcSwitchPayloadOverrideTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("payload_adapter", "sim");
    node_ = std::make_shared<runtime::FlightRuntimeNode>(options);
  }

  std::shared_ptr<runtime::FlightRuntimeNode> node_;
};

// 1. Test Edge Trigger: Down (Open) then Up (Lock)
TEST_F(RcSwitchPayloadOverrideTest, EdgeTriggerOpenThenLock)
{
  auto controller = node_->get_payload_controller();
  ASSERT_NE(controller, nullptr);

  // Initial Sync (UP)
  auto msg_sync = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
  msg_sync->valid = true;
  msg_sync->aux1 = -0.88f; // UP
  node_->handle_manual_control_setpoint(msg_sync);

  // 1. Flip DOWN -> OPEN
  auto msg_down = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
  msg_down->valid = true;
  msg_down->aux1 = 0.86f; // DOWN
  node_->handle_manual_control_setpoint(msg_down);

  auto status1 = controller->get_status();
  EXPECT_EQ(status1.commanded_state, msg::PayloadStatus::COMMAND_OPEN);
  EXPECT_EQ(status1.feedback_state, msg::PayloadStatus::FEEDBACK_OPEN);
  EXPECT_FALSE(controller->is_ready_for_sortie());

  // 2. Flip UP -> LOCK
  auto msg_up = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
  msg_up->valid = true;
  msg_up->aux1 = -0.88f; // UP
  node_->handle_manual_control_setpoint(msg_up);

  auto status2 = controller->get_status();
  EXPECT_EQ(status2.commanded_state, msg::PayloadStatus::COMMAND_SECURED);
  EXPECT_EQ(status2.feedback_state, msg::PayloadStatus::FEEDBACK_SECURED);
  EXPECT_TRUE(controller->is_ready_for_sortie());
}

// 2. Test Invariance: Static Switch Position does NOT overwrite Autonomous Release
TEST_F(RcSwitchPayloadOverrideTest, StaticSwitchInvarianceDuringAutonomousDrop)
{
  auto controller = node_->get_payload_controller();
  ASSERT_NE(controller, nullptr);

  // Pilot flipped UP before flight
  auto msg_sync = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
  msg_sync->valid = true;
  msg_sync->aux1 = 0.86f; // Start down
  node_->handle_manual_control_setpoint(msg_sync);

  auto msg_up = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
  msg_up->valid = true;
  msg_up->aux1 = -0.88f; // Flip UP to lock
  node_->handle_manual_control_setpoint(msg_up);

  EXPECT_EQ(controller->get_status().feedback_state, msg::PayloadStatus::FEEDBACK_SECURED);

  // Autonomous delivery touches down at target pad and releases cargo
  msg::PayloadStatus drop_status;
  controller->prepare(
    srv::PreparePayload::Request::OP_OPEN_FOR_LOADING,
    "auto_drop_001", 0, drop_status);
  EXPECT_EQ(controller->get_status().feedback_state, msg::PayloadStatus::FEEDBACK_OPEN);

  // Radio continues streaming static UP position (aux1 = -0.88f)
  for (int i = 0; i < 5; ++i) {
    node_->handle_manual_control_setpoint(msg_up);
  }

  // MUST REMAIN OPEN (No accidental lock!)
  EXPECT_EQ(controller->get_status().feedback_state, msg::PayloadStatus::FEEDBACK_OPEN);
}
