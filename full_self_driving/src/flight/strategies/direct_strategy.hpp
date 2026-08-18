#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/utils/geodesic.hpp>

#include "adapters/px4_state_cache.hpp"
#include "flight/internal_strategy.hpp"
#include "full_self_driving/msg/pad_record.hpp"
#include "persistence/persistence_manager.hpp"

namespace full_self_driving::flight
{

class DirectStrategy : public InternalStrategy
{
public:
  using CompletionCallback = std::function<void(bool /* success */)>;

  DirectStrategy(
    rclcpp::Node & node,
    px4_ros2::Context & context,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    double target_latitude_deg,
    double target_longitude_deg,
    double target_altitude_above_home_m = 5.0,
    float arrival_radius_m = 4.0f,
    float settle_speed_m_s = 0.5f,
    float settle_duration_s = 1.0f,
    float max_horizontal_speed_m_s = 5.0f,
    float max_yaw_rate_rad_s = 0.785398163f,  // 45 deg/s
    double direct_timeout_s = 30.0,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  // Overload for testing with custom/injected GotoGlobalSetpointType
  DirectStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    double target_latitude_deg,
    double target_longitude_deg,
    double target_altitude_above_home_m = 5.0,
    float arrival_radius_m = 4.0f,
    float settle_speed_m_s = 0.5f,
    float settle_duration_s = 1.0f,
    float max_horizontal_speed_m_s = 5.0f,
    float max_yaw_rate_rad_s = 0.785398163f,
    double direct_timeout_s = 30.0,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  // Convenience constructor from PadRecord
  DirectStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    const full_self_driving::msg::PadRecord & pad_record,
    double target_altitude_above_home_m = 5.0,
    float arrival_radius_m = 4.0f,
    float settle_speed_m_s = 0.5f,
    float settle_duration_s = 1.0f,
    float max_horizontal_speed_m_s = 5.0f,
    float max_yaw_rate_rad_s = 0.785398163f,
    double direct_timeout_s = 30.0,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  ~DirectStrategy() override = default;

  void set_completion_callback(CompletionCallback cb) { completion_cb_ = std::move(cb); }

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return mode_finished_ && !failed_; }
  bool is_failed() const { return failed_; }
  bool is_settled() const { return settled_; }
  StrategyType get_type() const override { return StrategyType::DIRECT; }
  std::string get_name() const override { return "DIRECT"; }

  double target_latitude_deg() const { return target_latitude_deg_; }
  double target_longitude_deg() const { return target_longitude_deg_; }
  double target_altitude_above_home_m() const { return target_altitude_above_home_m_; }
  double target_altitude_amsl_m() const { return target_altitude_amsl_m_; }
  const std::string & failure_reason() const { return failure_reason_; }

private:
  void fail(const std::string & reason);
  bool data_timed_out() const;
  std::optional<float> update_course_heading(const adapters::Px4StateSnapshot & snapshot);

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;

  double target_latitude_deg_{0.0};
  double target_longitude_deg_{0.0};
  double target_altitude_above_home_m_{5.0};
  double target_altitude_amsl_m_{0.0};
  double home_altitude_msl_m_{0.0};

  float arrival_radius_m_{4.0f};
  float settle_speed_m_s_{0.5f};
  float settle_duration_s_{1.0f};
  float max_horizontal_speed_m_s_{5.0f};
  float max_vertical_speed_m_s_{1.0f};
  float max_yaw_rate_rad_s_{0.785398163f};
  float altitude_tolerance_m_{1.0f};
  float data_timeout_s_{2.0f};
  double direct_timeout_s_{30.0};

  std::chrono::steady_clock::time_point activation_time_{};
  float settle_accumulated_s_{0.0f};

  bool parameters_valid_{true};
  bool target_altitude_set_{false};
  bool setpoint_sent_{false};
  bool settled_{false};
  bool mode_finished_{false};
  bool failed_{false};
  std::string failure_reason_;

  bool last_heading_valid_{false};
  float last_heading_rad_{0.0f};

  CompletionCallback completion_cb_;
};

}  // namespace full_self_driving::flight
