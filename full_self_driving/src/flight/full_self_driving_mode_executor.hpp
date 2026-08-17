#pragma once

#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode_executor.hpp>
#include "flight/full_self_driving_mode.hpp"

namespace full_self_driving::flight
{

class FullSelfDrivingModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  using TakeoverCallback = std::function<void(DeactivateReason)>;
  using ExecutorActivationCallback = std::function<void(bool /* is_active */)>;

  FullSelfDrivingModeExecutor(
    rclcpp::Node & node,
    FullSelfDrivingMode & owned_mode,
    const std::string & topic_namespace_prefix = "");

  ~FullSelfDrivingModeExecutor() override = default;

  void set_takeover_callback(TakeoverCallback cb) { takeover_cb_ = std::move(cb); }
  void set_activation_callback(ExecutorActivationCallback cb) { activation_cb_ = std::move(cb); }

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;
  void onFailsafeDeferred() override;

  bool is_active() const { return is_active_; }
  DeactivateReason last_deactivate_reason() const { return last_deactivate_reason_; }
  std::string last_deactivate_reason_string() const;

private:
  rclcpp::Node & node_;
  FullSelfDrivingMode & mode_;
  bool is_active_{false};
  DeactivateReason last_deactivate_reason_{DeactivateReason::Other};

  TakeoverCallback takeover_cb_;
  ExecutorActivationCallback activation_cb_;
};

}  // namespace full_self_driving::flight
