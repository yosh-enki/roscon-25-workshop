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
#include "domain/route.hpp"
#include "flight/internal_strategy.hpp"
#include "persistence/persistence_manager.hpp"

namespace full_self_driving::flight
{

class TransitInStrategy : public InternalStrategy
{
public:
  using WaypointCallback = std::function<void(std::size_t /* index */, bool /* success */)>;
  using CompletionCallback = std::function<void(bool /* success */)>;

  TransitInStrategy(
    rclcpp::Node & node,
    px4_ros2::Context & context,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    domain::Route route,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  // Overload for testing with custom/injected GotoGlobalSetpointType
  TransitInStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    domain::Route route,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  ~TransitInStrategy() override = default;

  void set_waypoint_callback(WaypointCallback cb) { waypoint_cb_ = std::move(cb); }
  void set_completion_callback(CompletionCallback cb) { completion_cb_ = std::move(cb); }

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return mode_finished_ && !failed_; }
  bool is_failed() const override { return failed_; }
  StrategyType get_type() const override { return StrategyType::TRANSIT_IN; }
  std::string get_name() const override { return "TRANSIT_IN"; }

  std::size_t current_waypoint_index() const { return waypoint_index_; }
  double target_altitude_msl_m() const { return target_altitude_msl_m_; }
  std::string failure_reason() const override { return failure_reason_; }

  const domain::Route & route() const { return route_; }

private:
  void fail(const std::string & reason);
  std::optional<float> update_course_heading(const adapters::Px4StateSnapshot & snapshot);
  bool waypoint_reached(
    const adapters::Px4StateSnapshot & snapshot,
    const Eigen::Vector3d & target) const;
  bool data_timed_out() const;

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  domain::Route route_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;

  std::vector<domain::RoutePoint> waypoints_;
  std::size_t waypoint_index_{0};

  double transit_altitude_above_home_m_{10.0};
  double target_altitude_msl_m_{0.0};
  double home_altitude_msl_m_{0.0};

  float arrival_radius_m_{2.0f};
  float max_horizontal_speed_m_s_{3.0f};
  float max_vertical_speed_m_s_{1.0f};
  float max_heading_rate_rad_s_{0.785398163f};
  float course_heading_min_speed_m_s_{0.3f};
  float altitude_tolerance_m_{1.0f};
  float altitude_settle_speed_m_s_{0.5f};
  float data_timeout_s_{2.0f};

  std::chrono::steady_clock::time_point activation_time_{};

  bool parameters_valid_{true};
  bool target_altitude_set_{false};
  bool setpoint_sent_for_current_waypoint_{false};
  bool mode_finished_{false};
  bool failed_{false};
  std::string failure_reason_;

  bool last_heading_valid_{false};
  float last_heading_rad_{0.0f};

  WaypointCallback waypoint_cb_;
  CompletionCallback completion_cb_;
};

}  // namespace full_self_driving::flight
