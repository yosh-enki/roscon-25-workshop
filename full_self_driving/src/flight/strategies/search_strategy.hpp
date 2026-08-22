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
#include "domain/working_plan.hpp"
#include "flight/internal_strategy.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/plan_manager.hpp"

namespace full_self_driving::flight
{

class SearchStrategy : public InternalStrategy
{
public:
  using WaypointCallback = std::function<void(std::size_t /* index */, bool /* success */)>;
  using CompletionCallback = std::function<void(bool /* success */)>;
  using CheckpointCallback = std::function<void(const domain::SearchCheckpointData &)>;

  SearchStrategy(
    rclcpp::Node & node,
    px4_ros2::Context & context,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    std::shared_ptr<runtime::PlanManager> plan_manager,
    domain::WorkingPlan working_plan,
    double search_altitude_m = 15.0,
    float max_horizontal_speed_m_s = 5.0f,
    float waypoint_reach_radius_m = 4.0f,
    float max_yaw_rate_rad_s = 0.785398163f,  // 45 deg/s
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  // Overload for testing with custom/injected GotoGlobalSetpointType
  SearchStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    std::shared_ptr<runtime::PlanManager> plan_manager,
    domain::WorkingPlan working_plan,
    double search_altitude_m = 15.0,
    float max_horizontal_speed_m_s = 5.0f,
    float waypoint_reach_radius_m = 4.0f,
    float max_yaw_rate_rad_s = 0.785398163f,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  // Convenience constructor taking CanonicalSearchRoute directly
  SearchStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    domain::CanonicalSearchRoute route,
    double search_altitude_m = 15.0,
    float max_horizontal_speed_m_s = 5.0f,
    float waypoint_reach_radius_m = 4.0f,
    float max_yaw_rate_rad_s = 0.785398163f);

  ~SearchStrategy() override = default;

  void set_waypoint_callback(WaypointCallback cb) { waypoint_cb_ = std::move(cb); }
  void set_completion_callback(CompletionCallback cb) { completion_cb_ = std::move(cb); }
  void set_checkpoint_callback(CheckpointCallback cb) { checkpoint_cb_ = std::move(cb); }

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return mode_finished_ && !failed_; }
  bool is_failed() const override { return failed_; }
  StrategyType get_type() const override { return StrategyType::SEARCH; }
  std::string get_name() const override { return "SEARCH"; }

  std::size_t current_waypoint_index() const { return current_waypoint_index_; }
  double search_altitude_m() const { return search_altitude_m_; }
  double target_altitude_amsl_m() const { return target_altitude_amsl_m_; }
  const domain::WorkingPlan & working_plan() const { return working_plan_; }
  const domain::CanonicalSearchRoute & route() const { return route_; }
  std::string failure_reason() const override { return failure_reason_; }

private:
  void fail(const std::string & reason);
  bool data_timed_out() const;
  void record_safe_deactivation_checkpoint();

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;

  domain::WorkingPlan working_plan_;
  domain::CanonicalSearchRoute route_;
  std::size_t current_waypoint_index_{0};

  double search_altitude_m_{15.0};
  double target_altitude_amsl_m_{0.0};
  double home_altitude_msl_m_{0.0};

  float waypoint_reach_radius_m_{4.0f};
  float max_horizontal_speed_m_s_{5.0f};
  float max_vertical_speed_m_s_{1.0f};
  float max_yaw_rate_rad_s_{0.785398163f};
  float altitude_tolerance_m_{1.0f};
  float data_timeout_s_{2.0f};

  std::chrono::steady_clock::time_point activation_time_{};

  bool parameters_valid_{true};
  bool target_altitude_set_{false};
  bool mode_finished_{false};
  bool failed_{false};
  std::string failure_reason_;

  bool starts_with_entry_point_{false};
  uint32_t first_plan_waypoint_source_index_{0};
  uint32_t total_source_waypoints_{0};

  WaypointCallback waypoint_cb_;
  CompletionCallback completion_cb_;
  CheckpointCallback checkpoint_cb_;
};

}  // namespace full_self_driving::flight
