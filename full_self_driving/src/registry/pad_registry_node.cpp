#include "registry/pad_registry_node.hpp"

#include <chrono>
#include <memory>
#include <utility>

namespace full_self_driving::registry
{

PadRegistryNode::PadRegistryNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fsd_pad_registry", options)
{
  load_parameters();
}

void PadRegistryNode::load_parameters()
{
  declare_parameter<std::string>("map_id", "kmitl_airfield");
  declare_parameter<std::string>("scenario_id", "default_scenario");
  declare_parameter<std::string>("all_id_observations_topic", "/full_self_driving/perception/all_id_observations");
  declare_parameter<std::string>("snapshot_topic", "/full_self_driving/pad_registry");
  declare_parameter<std::string>("status_topic", "/full_self_driving/pad_registry/status");
  declare_parameter<std::string>("health_topic", "/full_self_driving/health");
  declare_parameter<double>("min_quality", 0.0);
  declare_parameter<double>("max_record_age_s", 3600.0);
  declare_parameter<double>("max_record_uncertainty_m", 50.0);
  declare_parameter<bool>("autostart", false);

  get_parameter("map_id", map_id_);
  get_parameter("scenario_id", scenario_id_);
  get_parameter("all_id_observations_topic", all_id_observations_topic_);
  get_parameter("snapshot_topic", snapshot_topic_);
  get_parameter("status_topic", status_topic_);
  get_parameter("health_topic", health_topic_);

  double min_q = 0.0;
  get_parameter("min_quality", min_q);
  config_.min_quality = static_cast<float>(min_q);
  get_parameter("max_record_age_s", config_.max_record_age_s);
  get_parameter("max_record_uncertainty_m", config_.max_record_uncertainty_m);
  config_.default_map_id = map_id_;
  config_.default_scenario_id = scenario_id_;
  get_parameter("autostart", autostart_);

  registry_.set_config(config_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring fsd_pad_registry for map '%s', scenario '%s'",
    map_id_.c_str(), scenario_id_.c_str());

  // Status and Snapshot QoS: Reliable, Transient Local, depth 1
  auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
  auto reliable_qos = rclcpp::QoS(5).reliable();

  snapshot_pub_ = create_publisher<full_self_driving::msg::PadRegistrySnapshot>(
    snapshot_topic_, latched_qos);
  status_pub_ = create_publisher<full_self_driving::msg::PadRegistryStatus>(
    status_topic_, latched_qos);
  health_pub_ = create_publisher<full_self_driving::msg::ComponentHealth>(
    health_topic_, reliable_qos);

  timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PadRegistryNode::timer_callback, this));

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_INACTIVE,
    false,
    "Pad registry configured, inactive");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating fsd_pad_registry");

  snapshot_pub_->on_activate();
  status_pub_->on_activate();
  health_pub_->on_activate();

  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
  all_id_sub_ = create_subscription<full_self_driving::msg::AllIdObservationBatch>(
    all_id_observations_topic_, sensor_qos,
    std::bind(&PadRegistryNode::all_id_callback, this, std::placeholders::_1));

  publish_snapshot_and_status();

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_ACTIVE,
    true,
    "Pad registry active");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating fsd_pad_registry");

  all_id_sub_.reset();

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_INACTIVE,
    false,
    "Pad registry deactivated");

  snapshot_pub_->on_deactivate();
  status_pub_->on_deactivate();
  health_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up fsd_pad_registry");

  all_id_sub_.reset();
  timer_.reset();
  snapshot_pub_.reset();
  status_pub_.reset();
  health_pub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down fsd_pad_registry");

  all_id_sub_.reset();
  timer_.reset();
  snapshot_pub_.reset();
  status_pub_.reset();
  health_pub_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PadRegistryNode::all_id_callback(const full_self_driving::msg::AllIdObservationBatch::SharedPtr msg)
{
  uint64_t monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  size_t accepted = registry_.observe(*msg, monotonic_ns);
  if (accepted > 0) {
    publish_snapshot_and_status();
  }
}

void PadRegistryNode::timer_callback()
{
  publish_snapshot_and_status();

  uint8_t state = full_self_driving::msg::ComponentHealth::STATE_UNKNOWN;
  bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  if (is_active) {
    state = full_self_driving::msg::ComponentHealth::STATE_ACTIVE;
  } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    state = full_self_driving::msg::ComponentHealth::STATE_STARTING;
  } else {
    state = full_self_driving::msg::ComponentHealth::STATE_INACTIVE;
  }

  publish_health(state, is_active, is_active ? "Pad registry active" : "Inactive");
}

void PadRegistryNode::publish_snapshot_and_status()
{
  uint64_t monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());

  builtin_interfaces::msg::Time now_stamp = this->now();

  if (snapshot_pub_ && snapshot_pub_->is_activated()) {
    auto snapshot = registry_.get_snapshot(map_id_, scenario_id_, now_stamp, monotonic_ns);
    snapshot_pub_->publish(snapshot);
  }

  if (status_pub_ && status_pub_->is_activated()) {
    full_self_driving::msg::ComponentHealth h;
    h.component_id = "fsd_pad_registry";
    h.state = full_self_driving::msg::ComponentHealth::STATE_ACTIVE;
    h.ready = true;
    h.last_update_monotonic_ns = monotonic_ns;

    auto status = registry_.get_status(map_id_, scenario_id_, now_stamp, monotonic_ns, h, is_disarmed_);
    status_pub_->publish(status);
  }
}

void PadRegistryNode::publish_health(uint8_t state, bool ready, const std::string & detail)
{
  if (!health_pub_) {
    return;
  }

  full_self_driving::msg::ComponentHealth msg;
  msg.component_id = "fsd_pad_registry";
  msg.state = state;
  msg.ready = ready;
  msg.last_update_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  msg.queue_depth = 0;
  msg.queue_drop_count = 0;
  msg.detail = detail;

  if (health_pub_->is_activated()) {
    health_pub_->publish(msg);
  }
}

}  // namespace full_self_driving::registry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<full_self_driving::registry::PadRegistryNode>(options);

  bool autostart = false;
  node->get_parameter("autostart", autostart);

  if (autostart) {
    node->configure();
    node->activate();
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
