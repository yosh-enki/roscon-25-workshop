#include "registry/pad_registry_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  declare_parameter<std::string>("world_frame", "map");
  declare_parameter<std::string>("global_position_topic", "/fmu/out/vehicle_global_position");
  declare_parameter<std::string>("all_id_observations_topic", "/full_self_driving/perception/all_id_observations");
  declare_parameter<std::string>("snapshot_topic", "/full_self_driving/pad_registry");
  declare_parameter<std::string>("status_topic", "/full_self_driving/pad_registry/status");
  declare_parameter<std::string>("health_topic", "/full_self_driving/health");
  declare_parameter<double>("min_quality", 0.0);
  declare_parameter<double>("max_record_age_s", 3600.0);
  declare_parameter<double>("max_record_uncertainty_m", 50.0);
  declare_parameter<double>("transform_timeout_s", 0.2);
  declare_parameter<bool>("autostart", false);

  get_parameter("map_id", map_id_);
  get_parameter("scenario_id", scenario_id_);
  get_parameter("world_frame", world_frame_);
  get_parameter("global_position_topic", global_position_topic_);
  get_parameter("all_id_observations_topic", all_id_observations_topic_);
  get_parameter("snapshot_topic", snapshot_topic_);
  get_parameter("status_topic", status_topic_);
  get_parameter("health_topic", health_topic_);

  double min_q = 0.0;
  get_parameter("min_quality", min_q);
  config_.min_quality = static_cast<float>(min_q);
  get_parameter("max_record_age_s", config_.max_record_age_s);
  get_parameter("max_record_uncertainty_m", config_.max_record_uncertainty_m);
  get_parameter("transform_timeout_s", transform_timeout_s_);
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

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, true);

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

  // Dynamic PX4 GPS Auto-Origin subscription (Locks home datum once from live GPS)
  global_pos_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
    global_position_topic_, sensor_qos,
    [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
      if (!origin_ready_ && std::isfinite(msg->lat) && std::isfinite(msg->lon) &&
          (std::abs(msg->lat) > 0.0001 || std::abs(msg->lon) > 0.0001))
      {
        std::lock_guard<std::mutex> lock(origin_mutex_);
        origin_latitude_deg_ = msg->lat;
        origin_longitude_deg_ = msg->lon;
        origin_elevation_m_ = msg->alt;
        origin_ready_ = true;
        RCLCPP_INFO(get_logger(),
          "[fsd_pad_registry] Dynamic Map Origin locked from live PX4 GPS fix: Lat=%.7f, Lon=%.7f, Alt=%.2f m",
          origin_latitude_deg_, origin_longitude_deg_, origin_elevation_m_);
      }
    });

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
  global_pos_sub_.reset();

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
  global_pos_sub_.reset();
  timer_.reset();
  snapshot_pub_.reset();
  status_pub_.reset();
  health_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PadRegistryNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down fsd_pad_registry");

  all_id_sub_.reset();
  global_pos_sub_.reset();
  timer_.reset();
  snapshot_pub_.reset();
  status_pub_.reset();
  health_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PadRegistryNode::all_id_callback(const full_self_driving::msg::AllIdObservationBatch::SharedPtr msg)
{
  uint64_t monotonic_ns = this->get_clock()->now().nanoseconds();

  constexpr double kPi = 3.14159265358979323846;
  constexpr double kWgs84SemiMajorAxisM = 6378137.0;
  constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;

  double origin_lat = 0.0, origin_lon = 0.0, origin_alt = 0.0;
  bool has_origin = false;
  {
    std::lock_guard<std::mutex> lock(origin_mutex_);
    if (origin_ready_) {
      origin_lat = origin_latitude_deg_;
      origin_lon = origin_longitude_deg_;
      origin_alt = origin_elevation_m_;
      has_origin = true;
    }
  }

  full_self_driving::msg::AllIdObservationBatch transformed_batch = *msg;

  for (auto & obs : transformed_batch.observations) {
    if (tf_buffer_ && has_origin) {
      try {
        geometry_msgs::msg::TransformStamped world_transform = tf_buffer_->lookupTransform(
          world_frame_,
          obs.pose_frame,
          tf2::TimePointZero,
          tf2::durationFromSec(transform_timeout_s_));

        geometry_msgs::msg::PoseStamped src_pose;
        src_pose.header.frame_id = obs.pose_frame;
        src_pose.header.stamp = obs.image_time;
        src_pose.pose = obs.pose;

        geometry_msgs::msg::PoseStamped world_pose;
        tf2::doTransform(src_pose, world_pose, world_transform);

        const double north_m = world_pose.pose.position.x;
        const double east_m = world_pose.pose.position.y;

        const double origin_lat_rad = origin_lat * kPi / 180.0;
        const double sin_lat = std::sin(origin_lat_rad);
        const double sin_sq = sin_lat * sin_lat;
        const double prime_vertical_radius = kWgs84SemiMajorAxisM /
          std::sqrt(1.0 - kWgs84EccentricitySquared * sin_sq);
        const double meridian_radius = kWgs84SemiMajorAxisM *
          (1.0 - kWgs84EccentricitySquared) /
          std::pow(1.0 - kWgs84EccentricitySquared * sin_sq, 1.5);
        const double cos_lat = std::cos(origin_lat_rad);

        if (std::isfinite(north_m) && std::isfinite(east_m) && std::abs(cos_lat) > 1e-9) {
          double lat = origin_lat + (north_m / meridian_radius) * 180.0 / kPi;
          double lon = origin_lon + (east_m / (prime_vertical_radius * cos_lat)) * 180.0 / kPi;
          double alt = origin_alt + world_pose.pose.position.z;

          obs.pose.position.x = lat;
          obs.pose.position.y = lon;
          obs.pose.position.z = alt;
        }
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Could not transform observation frame '%s' to '%s': %s",
          obs.pose_frame.c_str(), world_frame_.c_str(), error.what());
      }
    }
  }

  size_t accepted = registry_.observe(transformed_batch, monotonic_ns);
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
  uint64_t monotonic_ns = this->get_clock()->now().nanoseconds();
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
  msg.last_update_monotonic_ns = this->get_clock()->now().nanoseconds();
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
