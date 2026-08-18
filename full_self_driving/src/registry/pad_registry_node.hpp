#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

#include "full_self_driving/msg/all_id_observation_batch.hpp"
#include "full_self_driving/msg/component_health.hpp"
#include "full_self_driving/msg/pad_registry_snapshot.hpp"
#include "full_self_driving/msg/pad_registry_status.hpp"
#include "registry/pad_registry.hpp"

namespace full_self_driving::registry
{

class PadRegistryNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PadRegistryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PadRegistryNode() override = default;

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

  PadRegistry & get_registry() { return registry_; }

private:
  void load_parameters();
  void all_id_callback(const full_self_driving::msg::AllIdObservationBatch::SharedPtr msg);
  void timer_callback();
  void publish_snapshot_and_status();
  void publish_health(uint8_t state, bool ready, const std::string & detail);

  PadRegistry registry_;
  RegistryConfig config_;

  std::string map_id_{"kmitl_airfield"};
  std::string scenario_id_{"default_scenario"};
  std::string world_frame_{"map"};
  std::string global_position_topic_{"/fmu/out/vehicle_global_position"};
  double transform_timeout_s_{0.2};

  // Dynamic Map Origin (Locked once from live PX4 GPS fix on boot)
  bool origin_ready_{false};
  double origin_latitude_deg_{0.0};
  double origin_longitude_deg_{0.0};
  double origin_elevation_m_{0.0};
  std::mutex origin_mutex_;

  std::string all_id_observations_topic_{"/full_self_driving/perception/all_id_observations"};
  std::string snapshot_topic_{"/full_self_driving/pad_registry"};
  std::string status_topic_{"/full_self_driving/pad_registry/status"};
  std::string health_topic_{"/full_self_driving/health"};
  bool autostart_{false};
  bool is_disarmed_{true};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<full_self_driving::msg::AllIdObservationBatch>::SharedPtr all_id_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_sub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::PadRegistrySnapshot>> snapshot_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::PadRegistryStatus>> status_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::ComponentHealth>> health_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace full_self_driving::registry
