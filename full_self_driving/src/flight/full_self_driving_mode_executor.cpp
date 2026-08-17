#include "flight/full_self_driving_mode_executor.hpp"

namespace full_self_driving::flight
{

FullSelfDrivingModeExecutor::FullSelfDrivingModeExecutor(
  rclcpp::Node & node,
  FullSelfDrivingMode & owned_mode,
  const std::string & topic_namespace_prefix)
: px4_ros2::ModeExecutorBase(
    node,
    ModeExecutorBase::Settings{ModeExecutorBase::Settings::Activation::ActivateAlways},
    owned_mode,
    topic_namespace_prefix),
  node_(node),
  mode_(owned_mode)
{
}

void FullSelfDrivingModeExecutor::onActivate()
{
  is_active_ = true;
  RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] FullSelfDrivingModeExecutor activated (is_in_charge=%d)", isInCharge());
  if (activation_cb_) {
    activation_cb_(true);
  }
}

void FullSelfDrivingModeExecutor::onDeactivate(DeactivateReason reason)
{
  is_active_ = false;
  last_deactivate_reason_ = reason;
  RCLCPP_WARN(node_.get_logger(), "[EXECUTOR] FullSelfDrivingModeExecutor deactivated (reason=%s)",
    last_deactivate_reason_string().c_str());

  if (activation_cb_) {
    activation_cb_(false);
  }
  if (takeover_cb_) {
    takeover_cb_(reason);
  }
}

void FullSelfDrivingModeExecutor::onFailsafeDeferred()
{
  RCLCPP_WARN(node_.get_logger(), "[EXECUTOR] Failsafe deferred event received");
}

std::string FullSelfDrivingModeExecutor::last_deactivate_reason_string() const
{
  switch (last_deactivate_reason_) {
    case DeactivateReason::FailsafeActivated: return "FailsafeActivated";
    case DeactivateReason::Other: return "Other";
  }
  return "Unknown";
}

}  // namespace full_self_driving::flight
