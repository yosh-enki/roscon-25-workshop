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
#include "full_self_driving/msg/live_target_lock.hpp"
#include "full_self_driving/msg/target_identity.hpp"
#include "perception/aruco_detector.hpp"
#include "perception/target_coordinator.hpp"

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

  TargetCoordinator & get_target_coordinator() { return target_coordinator_; }

private:
  void load_parameters();
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void target_selection_callback(const full_self_driving::msg::TargetIdentity::SharedPtr msg);
  void health_timer_callback();
  void publish_health(uint8_t state, bool ready, const std::string & detail);

  std::unique_ptr<ArucoDetector> detector_;
  DetectorConfig config_;
  TargetCoordinator target_coordinator_;
  domain::TargetLockPolicy target_lock_policy_;

  std::string camera_topic_{"/camera"};
  std::string camera_info_topic_{"/camera_info"};
  std::string annotated_image_topic_{"/full_self_driving/perception/annotated_image"};
  std::string all_id_observations_topic_{"/full_self_driving/perception/all_id_observations"};
  std::string live_target_lock_topic_{"/full_self_driving/perception/live_target_lock"};
  std::string target_selection_topic_{"/full_self_driving/target_selection"};
  std::string health_topic_{"/full_self_driving/health"};
  bool autostart_{false};

  // Optional initial selected target from parameter
  int initial_selected_marker_id_{-1};
  std::string initial_selected_dictionary_{"DICT_4X4_50"};
  std::string initial_selected_namespace_{"aavc2026"};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<full_self_driving::msg::TargetIdentity>::SharedPtr target_selection_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::AllIdObservationBatch>> all_id_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> annotated_image_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::LiveTargetLock>> live_target_lock_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::ComponentHealth>> health_pub_;

  rclcpp::TimerBase::SharedPtr health_timer_;

  uint64_t sequence_{0};
  uint32_t queue_drops_{0};
  uint64_t last_target_selection_steady_ns_{0};
};

}  // namespace full_self_driving::perception
