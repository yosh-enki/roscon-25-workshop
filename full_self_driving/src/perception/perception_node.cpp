#include "perception/perception_node.hpp"

#include <chrono>
#include <memory>
#include <utility>

namespace full_self_driving::perception
{

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("fsd_perception", options)
{
  load_parameters();
}

void PerceptionNode::load_parameters()
{
  declare_parameter<std::string>("camera_topic", "/camera");
  declare_parameter<std::string>("camera_info_topic", "/camera_info");
  declare_parameter<std::string>("annotated_image_topic", "/full_self_driving/perception/annotated_image");
  declare_parameter<std::string>("all_id_observations_topic", "/full_self_driving/perception/all_id_observations");
  declare_parameter<std::string>("health_topic", "/full_self_driving/health");
  declare_parameter<std::string>("camera_frame", "camera_frame");
  declare_parameter<std::string>("map_id", "kmitl_airfield");
  declare_parameter<std::string>("scenario_id", "default_scenario");
  declare_parameter<std::string>("target_namespace", "aavc2026");
  declare_parameter<std::string>("dictionary", "DICT_4X4_50");
  declare_parameter<double>("marker_size", 0.4);
  declare_parameter<double>("min_quality", 0.0);
  declare_parameter<bool>("autostart", false);

  get_parameter("camera_topic", camera_topic_);
  get_parameter("camera_info_topic", camera_info_topic_);
  get_parameter("annotated_image_topic", annotated_image_topic_);
  get_parameter("all_id_observations_topic", all_id_observations_topic_);
  get_parameter("health_topic", health_topic_);
  get_parameter("camera_frame", config_.camera_frame);
  get_parameter("map_id", config_.map_id);
  get_parameter("scenario_id", config_.scenario_id);
  get_parameter("target_namespace", config_.target_namespace);
  get_parameter("dictionary", config_.dictionary_name);
  get_parameter("marker_size", config_.marker_size_m);
  double min_q = 0.0;
  get_parameter("min_quality", min_q);
  config_.min_quality = static_cast<float>(min_q);
  get_parameter("autostart", autostart_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring fsd_perception with dictionary %s, marker_size %.2f m",
    config_.dictionary_name.c_str(), config_.marker_size_m);

  detector_ = std::make_unique<ArucoDetector>(config_);

  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
  auto reliable_qos = rclcpp::QoS(5).reliable();

  all_id_pub_ = create_publisher<full_self_driving::msg::AllIdObservationBatch>(
    all_id_observations_topic_, sensor_qos);
  annotated_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    annotated_image_topic_, sensor_qos);
  health_pub_ = create_publisher<full_self_driving::msg::ComponentHealth>(
    health_topic_, reliable_qos);

  health_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PerceptionNode::health_timer_callback, this));

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_INACTIVE,
    false,
    "Perception configured, inactive");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating fsd_perception");

  all_id_pub_->on_activate();
  annotated_image_pub_->on_activate();
  health_pub_->on_activate();

  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, sensor_qos,
    std::bind(&PerceptionNode::image_callback, this, std::placeholders::_1));

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_, sensor_qos,
    std::bind(&PerceptionNode::camera_info_callback, this, std::placeholders::_1));

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_ACTIVE,
    detector_ && detector_->is_calibrated(),
    "Perception active, listening for camera feed");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating fsd_perception");

  image_sub_.reset();
  camera_info_sub_.reset();

  publish_health(
    full_self_driving::msg::ComponentHealth::STATE_INACTIVE,
    false,
    "Perception deactivated");

  all_id_pub_->on_deactivate();
  annotated_image_pub_->on_deactivate();
  health_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up fsd_perception");

  image_sub_.reset();
  camera_info_sub_.reset();
  health_timer_.reset();
  all_id_pub_.reset();
  annotated_image_pub_.reset();
  health_pub_.reset();
  detector_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PerceptionNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down fsd_perception");

  image_sub_.reset();
  camera_info_sub_.reset();
  health_timer_.reset();
  all_id_pub_.reset();
  annotated_image_pub_.reset();
  health_pub_.reset();
  detector_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PerceptionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!detector_) {
    return;
  }

  bool was_calibrated = detector_->is_calibrated();
  bool ok = detector_->update_camera_info(*msg);

  if (ok && !was_calibrated) {
    RCLCPP_INFO(get_logger(), "Camera calibration accepted. Hash: %s",
      detector_->get_calibration_hash().c_str());
  } else if (!ok) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received invalid camera info");
  }
}

void PerceptionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!detector_) {
    return;
  }

  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    uint64_t monotonic_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());

    DetectionResult res = detector_->process_image(
      cv_ptr->image,
      msg->header.stamp,
      monotonic_ns,
      ++sequence_,
      queue_drops_);

    if (all_id_pub_ && all_id_pub_->is_activated()) {
      all_id_pub_->publish(res.batch);
    }

    if (annotated_image_pub_ && annotated_image_pub_->is_activated()) {
      cv_bridge::CvImage out_img;
      out_img.header = msg->header;
      out_img.encoding = sensor_msgs::image_encodings::BGR8;
      out_img.image = res.annotated_image;
      annotated_image_pub_->publish(*out_img.toImageMsg());
    }

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge exception: %s", e.what());
  } catch (const cv::Exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "OpenCV exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Image processing exception: %s", e.what());
  }
}

void PerceptionNode::health_timer_callback()
{
  uint8_t state = full_self_driving::msg::ComponentHealth::STATE_UNKNOWN;
  bool is_active = (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  if (is_active) {
    state = full_self_driving::msg::ComponentHealth::STATE_ACTIVE;
  } else if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    state = full_self_driving::msg::ComponentHealth::STATE_STARTING;
  } else {
    state = full_self_driving::msg::ComponentHealth::STATE_INACTIVE;
  }

  bool is_calibrated = (detector_ && detector_->is_calibrated());
  bool ready = (is_active && is_calibrated);

  std::string detail = is_active ?
    (is_calibrated ? "Active and calibrated" : "Active, awaiting camera calibration") :
    "Inactive";

  publish_health(state, ready, detail);
}

void PerceptionNode::publish_health(uint8_t state, bool ready, const std::string & detail)
{
  if (!health_pub_) {
    return;
  }

  full_self_driving::msg::ComponentHealth msg;
  msg.component_id = "fsd_perception";
  msg.state = state;
  msg.ready = ready;
  msg.last_update_monotonic_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  msg.queue_depth = 0;
  msg.queue_drop_count = queue_drops_;
  msg.detail = detail;

  // LifecyclePublisher will only transmit if active, but if not yet active we don't throw
  if (health_pub_->is_activated()) {
    health_pub_->publish(msg);
  }
}

}  // namespace full_self_driving::perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = std::make_shared<full_self_driving::perception::PerceptionNode>(options);

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
