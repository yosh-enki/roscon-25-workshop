#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>

#include "adapters/px4_state_cache.hpp"
#include "flight/internal_strategy.hpp"

namespace full_self_driving::flight
{

class FullSelfDrivingMode : public px4_ros2::ModeBase
{
public:
  using ReadinessCheckCallback = std::function<bool(std::vector<std::string> & /* failure_codes */)>;
  using ActivationCallback = std::function<void(bool /* is_active */)>;

  FullSelfDrivingMode(
    rclcpp::Node & node,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    const std::string & topic_namespace_prefix = "");

  ~FullSelfDrivingMode() override = default;

  void set_readiness_check_callback(ReadinessCheckCallback cb) { readiness_cb_ = std::move(cb); }
  void set_activation_callback(ActivationCallback cb) { activation_cb_ = std::move(cb); }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter) override;
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

  void set_strategy(std::unique_ptr<InternalStrategy> strategy);
  StrategyType get_current_strategy_type() const;
  std::string get_current_strategy_name() const;

  std::shared_ptr<adapters::Px4StateCache> state_cache() const { return state_cache_; }

private:
  rclcpp::Node & node_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::unique_ptr<InternalStrategy> strategy_;
  std::shared_ptr<px4_ros2::GotoSetpointType> goto_setpoint_;

  ReadinessCheckCallback readiness_cb_;
  ActivationCallback activation_cb_;
};

}  // namespace full_self_driving::flight
