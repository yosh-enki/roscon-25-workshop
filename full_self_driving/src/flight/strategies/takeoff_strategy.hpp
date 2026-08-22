#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>

#include "adapters/px4_state_cache.hpp"
#include "flight/internal_strategy.hpp"

namespace full_self_driving::flight
{

class TakeoffStrategy : public InternalStrategy
{
public:
  using CompletionCallback = std::function<void(bool /* success */)>;

  TakeoffStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    double target_altitude_above_home_m = 10.0,
    double altitude_tolerance_m = 1.0,
    double settle_speed_m_s = 0.5,
    double timeout_s = 30.0,
    StrategyType strategy_type = StrategyType::TAKEOFF);

  ~TakeoffStrategy() override = default;

  void set_completion_callback(CompletionCallback cb) { completion_cb_ = std::move(cb); }

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return completed_; }
  bool is_failed() const override { return failed_; }
  StrategyType get_type() const override { return strategy_type_; }
  std::string get_name() const override {
    return strategy_type_ == StrategyType::TAKEOFF_AFTER_DELIVERY ? "TAKEOFF_AFTER_DELIVERY" : "TAKEOFF";
  }

  double target_altitude_above_home_m() const { return target_altitude_above_home_m_; }
  double target_altitude_msl_m() const { return target_altitude_msl_m_; }
  std::string failure_reason() const override { return failure_reason_; }

private:
  void fail(const std::string & reason);

  rclcpp::Node & node_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;

  double target_altitude_above_home_m_{10.0};
  double altitude_tolerance_m_{1.0};
  double settle_speed_m_s_{0.5};
  double timeout_s_{30.0};

  double home_altitude_msl_m_{0.0};
  double target_altitude_msl_m_{0.0};
  bool target_altitude_set_{false};

  bool completed_{false};
  bool failed_{false};
  std::string failure_reason_;
  int settle_cycles_{0};
  static constexpr int kRequiredSettleCycles = 3;

  std::chrono::steady_clock::time_point start_time_{};
  CompletionCallback completion_cb_;
  StrategyType strategy_type_{StrategyType::TAKEOFF};
};

}  // namespace full_self_driving::flight
