#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "full_self_driving/msg/component_health.hpp"
#include "full_self_driving/msg/message_header.hpp"

namespace full_self_driving::runtime
{

class EvidenceNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit EvidenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~EvidenceNode() override = default;

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

  const std::string & get_evidence_directory() const { return evidence_directory_; }
  uint64_t get_recorded_events_count() const { return event_count_; }

private:
  void publish_health();

  std::string evidence_directory_;
  uint64_t event_count_{0};
  std::mutex mutex_;

  rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::ComponentHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace full_self_driving::runtime
