#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

#include "full_self_driving/msg/all_id_observation_batch.hpp"
#include "full_self_driving/msg/component_health.hpp"
#include "perception/aruco_detector.hpp"

namespace full_self_driving::perception
{

class PerceptionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PerceptionNode() override = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void load_parameters();
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void health_timer_callback();
  void publish_health(uint8_t state, bool ready, const std::string & detail);

  std::unique_ptr<ArucoDetector> detector_;
  DetectorConfig config_;

  std::string camera_topic_{"/camera"};
  std::string camera_info_topic_{"/camera_info"};
  std::string annotated_image_topic_{"/full_self_driving/perception/annotated_image"};
  std::string all_id_observations_topic_{"/full_self_driving/perception/all_id_observations"};
  std::string health_topic_{"/full_self_driving/health"};
  bool autostart_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::AllIdObservationBatch>> all_id_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> annotated_image_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::ComponentHealth>> health_pub_;

  rclcpp::TimerBase::SharedPtr health_timer_;

  uint64_t sequence_{0};
  uint32_t queue_drops_{0};
};

}  // namespace full_self_driving::perception
