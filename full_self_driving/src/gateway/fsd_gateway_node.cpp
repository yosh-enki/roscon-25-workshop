#include "gateway/fsd_gateway_node.hpp"

namespace full_self_driving::gateway
{

GatewayNode::GatewayNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fsd_gateway", options)
{
  declare_parameter("autostart", false);
  declare_parameter("max_payload_bytes", 8388608);
  declare_parameter("max_request_age_s", 30.0);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GatewayNode::on_configure(const rclcpp_lifecycle::State &)
{
  GatewaySecurityPolicy policy;
  policy.max_payload_bytes = static_cast<uint64_t>(get_parameter("max_payload_bytes").as_int());
  policy.max_request_age_s = get_parameter("max_request_age_s").as_double();
  gateway_.set_policy(policy);

  health_pub_ = create_publisher<full_self_driving::msg::ComponentHealth>(
    "/full_self_driving/health", rclcpp::QoS(5));

  RCLCPP_INFO(get_logger(), "Configured fsd_gateway node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GatewayNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (health_pub_) {
    health_pub_->on_activate();
  }

  health_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GatewayNode::publish_health, this));

  publish_health();
  RCLCPP_INFO(get_logger(), "Activated fsd_gateway node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GatewayNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (health_timer_) {
    health_timer_->cancel();
    health_timer_.reset();
  }

  if (health_pub_) {
    health_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated fsd_gateway node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GatewayNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  health_pub_.reset();
  health_timer_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up fsd_gateway node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GatewayNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  health_pub_.reset();
  health_timer_.reset();
  RCLCPP_INFO(get_logger(), "Shut down fsd_gateway node");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GatewayNode::publish_health()
{
  if (!health_pub_ || !health_pub_->is_activated()) {
    return;
  }

  full_self_driving::msg::ComponentHealth health;
  health.component_id = "fsd_gateway";
  health.state = full_self_driving::msg::ComponentHealth::STATE_ACTIVE;
  health.ready = true;
  health.last_update_monotonic_ns = static_cast<uint64_t>(now().nanoseconds());
  health.queue_depth = 0;
  health.queue_drop_count = static_cast<uint32_t>(gateway_.get_security_violations_count());
  health.detail = "Gateway active, violations=" + std::to_string(gateway_.get_security_violations_count());

  health_pub_->publish(health);
}

}  // namespace full_self_driving::gateway

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<full_self_driving::gateway::GatewayNode>(options);

  if (node->has_parameter("autostart") && node->get_parameter("autostart").as_bool()) {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
