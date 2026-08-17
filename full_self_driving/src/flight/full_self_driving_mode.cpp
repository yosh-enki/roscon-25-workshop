#include "flight/full_self_driving_mode.hpp"

namespace full_self_driving::flight
{

FullSelfDrivingMode::FullSelfDrivingMode(
  rclcpp::Node & node,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  const std::string & topic_namespace_prefix)
: px4_ros2::ModeBase(node, Settings("Full Self-Driving", true), topic_namespace_prefix),
  node_(node),
  state_cache_(std::move(state_cache)),
  strategy_(std::make_unique<WaitingForModeStrategy>())
{
  goto_setpoint_ = std::make_shared<px4_ros2::GotoSetpointType>(*this);

  // Configure mode requirements
  modeRequirements().angular_velocity = true;
  modeRequirements().attitude = true;
  modeRequirements().local_position = true;
  modeRequirements().global_position = true;
  modeRequirements().home_position = true;
}

void FullSelfDrivingMode::checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter)
{
  if (readiness_cb_) {
    std::vector<std::string> failure_codes;
    if (!readiness_cb_(failure_codes)) {
      std::string detail = "Readiness prerequisites failed";
      if (!failure_codes.empty()) {
        detail += ": " + failure_codes[0];
      }
      reporter.armingCheckFailureExt(
        px4_ros2::events::ID("fsd_arming_check_failed"),
        px4_ros2::events::Log::Warning,
        detail.c_str());
    }
  }
}

void FullSelfDrivingMode::onActivate()
{
  RCLCPP_INFO(node_.get_logger(), "[MODE] FullSelfDrivingMode activated");
  if (strategy_) {
    strategy_->on_enter();
  }
  if (activation_cb_) {
    activation_cb_(true);
  }
}

void FullSelfDrivingMode::onDeactivate()
{
  RCLCPP_INFO(node_.get_logger(), "[MODE] FullSelfDrivingMode deactivated");
  if (strategy_) {
    strategy_->on_exit();
  }
  if (activation_cb_) {
    activation_cb_(false);
  }
}

void FullSelfDrivingMode::updateSetpoint(float dt_s)
{
  if (strategy_) {
    strategy_->on_update(dt_s);
  }
}

void FullSelfDrivingMode::set_strategy(std::unique_ptr<InternalStrategy> strategy)
{
  if (!strategy) {
    return;
  }
  if (isActive() && strategy_) {
    strategy_->on_exit();
  }
  strategy_ = std::move(strategy);
  if (isActive() && strategy_) {
    strategy_->on_enter();
  }
}

StrategyType FullSelfDrivingMode::get_current_strategy_type() const
{
  if (strategy_) {
    return strategy_->get_type();
  }
  return StrategyType::WAITING_FOR_MODE;
}

std::string FullSelfDrivingMode::get_current_strategy_name() const
{
  if (strategy_) {
    return strategy_->get_name();
  }
  return "UNKNOWN";
}

}  // namespace full_self_driving::flight
