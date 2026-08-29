#pragma once

#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode_executor.hpp>
#include "domain/mission_context.hpp"
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
    std::shared_ptr<adapters::Px4StateCache> state_cache = nullptr,
    std::shared_ptr<domain::MissionContext> mission_ctx = nullptr,
    const std::string & topic_namespace_prefix = "");

  ~FullSelfDrivingModeExecutor() override = default;

  void set_takeover_callback(TakeoverCallback cb) { takeover_cb_ = std::move(cb); }
  void set_activation_callback(ExecutorActivationCallback cb) { activation_cb_ = std::move(cb); }
  void set_takeoff_altitude(float alt) { takeoff_altitude_ = alt; }
  float takeoff_altitude() const { return takeoff_altitude_; }
  void set_state_cache(std::shared_ptr<adapters::Px4StateCache> sc) { state_cache_ = std::move(sc); }
  void set_mission_context(std::shared_ptr<domain::MissionContext> ctx) { mission_ctx_ = std::move(ctx); }

  void trigger_takeoff_sequence();

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
  float takeoff_altitude_{10.0f};
  DeactivateReason last_deactivate_reason_{DeactivateReason::Other};

  TakeoverCallback takeover_cb_;
  ExecutorActivationCallback activation_cb_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_{nullptr};
};

}  // namespace full_self_driving::flight
