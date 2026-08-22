#include "flight/full_self_driving_mode_executor.hpp"

namespace full_self_driving::flight
{

FullSelfDrivingModeExecutor::FullSelfDrivingModeExecutor(
  rclcpp::Node & node,
  FullSelfDrivingMode & owned_mode,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  const std::string & topic_namespace_prefix)
: px4_ros2::ModeExecutorBase(
    node,
    ModeExecutorBase::Settings{ModeExecutorBase::Settings::Activation::ActivateOnlyWhenArmed},
    owned_mode,
    topic_namespace_prefix),
  node_(node),
  mode_(owned_mode),
  state_cache_(std::move(state_cache))
{
}

void FullSelfDrivingModeExecutor::trigger_takeoff_sequence()
{
  float target_amsl = takeoff_altitude_;
  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.home_pos_valid) {
      target_amsl = static_cast<float>(snapshot.home_global_position.z()) + takeoff_altitude_;
      RCLCPP_INFO(
        node_.get_logger(),
        "[EXECUTOR] Starting relative takeoff: Home AMSL = %.2f m, Target relative = +%.2f m -> Target AMSL = %.2f m",
        snapshot.home_global_position.z(), takeoff_altitude_, target_amsl);
    } else if (snapshot.global_pos_valid) {
      target_amsl = static_cast<float>(snapshot.global_position.z()) + takeoff_altitude_;
      RCLCPP_INFO(
        node_.get_logger(),
        "[EXECUTOR] Starting relative takeoff: Ground AMSL = %.2f m, Target relative = +%.2f m -> Target AMSL = %.2f m",
        snapshot.global_position.z(), takeoff_altitude_, target_amsl);
    } else {
      RCLCPP_WARN(
        node_.get_logger(),
        "[EXECUTOR] Starting takeoff with nominal altitude %.2f m AMSL (no position lock yet)",
        target_amsl);
    }
  } else {
    RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Starting PX4 takeoff sequence to altitude %.2f m", takeoff_altitude_);
  }

  takeoff([this](px4_ros2::Result result) {
    if (result == px4_ros2::Result::Success) {
      RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Takeoff completed successfully. Scheduling owned mode (id=%d)...", mode_.id());
      scheduleMode(mode_.id(), [this](px4_ros2::Result mode_result) {
        RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Owned mode completed with result: %s",
          px4_ros2::resultToString(mode_result));
      });
    } else {
      RCLCPP_ERROR(node_.get_logger(), "[EXECUTOR] Takeoff failed with result: %s",
        px4_ros2::resultToString(result));
    }
  }, target_amsl);
}

void FullSelfDrivingModeExecutor::onActivate()
{
  is_active_ = true;
  RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] FullSelfDrivingModeExecutor activated (is_in_charge=%d, is_armed=%d)",
    isInCharge(), isArmed());
  if (activation_cb_) {
    activation_cb_(true);
  }

  if (isInCharge()) {
    std::vector<std::string> failure_codes;
    if (!mode_.is_ready(failure_codes)) {
      std::string reason = failure_codes.empty() ? "Readiness prerequisites not met" : failure_codes[0];
      RCLCPP_ERROR(node_.get_logger(),
        "[EXECUTOR] Activation rejected: Mode readiness prerequisites failed (%s). Takeoff aborted.",
        reason.c_str());
      return;
    }

    if (!isArmed()) {
      RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Vehicle disarmed. Arming vehicle before takeoff...");
      arm([this](px4_ros2::Result result) {
        if (result == px4_ros2::Result::Success) {
          RCLCPP_INFO(node_.get_logger(), "[EXECUTOR] Arming succeeded. Triggering takeoff...");
          trigger_takeoff_sequence();
        } else {
          RCLCPP_ERROR(node_.get_logger(), "[EXECUTOR] Arming failed with result: %s",
            px4_ros2::resultToString(result));
        }
      });
    } else {
      trigger_takeoff_sequence();
    }
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
