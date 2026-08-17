#include "runtime/evidence_node.hpp"

#include <filesystem>

namespace full_self_driving::runtime
{

EvidenceNode::EvidenceNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fsd_evidence", options)
{
  declare_parameter("evidence_directory", "/tmp/fsd_evidence");
  declare_parameter("autostart", false);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EvidenceNode::on_configure(const rclcpp_lifecycle::State &)
{
  evidence_directory_ = get_parameter("evidence_directory").as_string();

  std::error_code ec;
  std::filesystem::create_directories(evidence_directory_, ec);

  health_pub_ = create_publisher<full_self_driving::msg::ComponentHealth>(
    "/full_self_driving/health", rclcpp::QoS(5));

  RCLCPP_INFO(get_logger(), "Configured fsd_evidence with directory '%s'", evidence_directory_.c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EvidenceNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (health_pub_) {
    health_pub_->on_activate();
  }

  health_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&EvidenceNode::publish_health, this));

  publish_health();
  RCLCPP_INFO(get_logger(), "Activated fsd_evidence");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EvidenceNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (health_timer_) {
    health_timer_->cancel();
    health_timer_.reset();
  }

  if (health_pub_) {
    health_pub_->on_deactivate();
  }

  RCLCPP_INFO(get_logger(), "Deactivated fsd_evidence");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EvidenceNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  health_pub_.reset();
  health_timer_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up fsd_evidence");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EvidenceNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  health_pub_.reset();
  health_timer_.reset();
  RCLCPP_INFO(get_logger(), "Shut down fsd_evidence");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EvidenceNode::publish_health()
{
  if (!health_pub_ || !health_pub_->is_activated()) {
    return;
  }

  full_self_driving::msg::ComponentHealth health;
  health.component_id = "fsd_evidence";
  health.state = full_self_driving::msg::ComponentHealth::STATE_ACTIVE;
  health.ready = true;
  health.last_update_monotonic_ns = static_cast<uint64_t>(now().nanoseconds());
  health.queue_depth = 0;
  health.queue_drop_count = 0;
  health.detail = "Evidence node operational";

  health_pub_->publish(health);
}

}  // namespace full_self_driving::runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<full_self_driving::runtime::EvidenceNode>(options);

  if (node->has_parameter("autostart") && node->get_parameter("autostart").as_bool()) {
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
