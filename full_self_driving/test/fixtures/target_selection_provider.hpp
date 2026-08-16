#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "full_self_driving/msg/target_identity.hpp"

namespace full_self_driving::test
{

class TargetSelectionProvider : public rclcpp::Node
{
public:
  explicit TargetSelectionProvider(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void set_target(uint32_t marker_id, const std::string & dictionary, const std::string & target_namespace);
  void publish_selection();

private:
  uint32_t marker_id_{0};
  std::string dictionary_{"DICT_4X4_50"};
  std::string target_namespace_{"aavc2026"};
  double rate_hz_{1.0};
  bool periodic_{true};

  rclcpp::Publisher<full_self_driving::msg::TargetIdentity>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace full_self_driving::test
