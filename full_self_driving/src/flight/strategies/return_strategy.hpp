#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/geodesic.hpp>

#include "adapters/px4_state_cache.hpp"
#include "domain/mission_context.hpp"
#include "flight/internal_strategy.hpp"
#include "persistence/persistence_manager.hpp"

namespace full_self_driving::flight
{

class ReturnStrategy : public InternalStrategy
{
public:
  enum class ReturnMode
  {
    RETURN_TO_HOME = 0,
    LAND_IMMEDIATELY = 1,
    HOLD_AT_FINAL_WAYPOINT = 2
  };

  enum class SubPhase
  {
    APPROACH_HOME = 0,
    DESCEND_HOME = 1,
    TOUCHDOWN_DWELL = 2,
    FINISHED = 3
  };

  ReturnStrategy(
    rclcpp::Node & node,
    px4_ros2::Context & context,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr,
    std::shared_ptr<domain::MissionContext> mission_ctx = nullptr,
    ReturnMode return_mode = ReturnMode::RETURN_TO_HOME,
    double return_altitude_above_home_m = 15.0);

  ReturnStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<px4_ros2::TrajectorySetpointType> traj_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr,
    std::shared_ptr<domain::MissionContext> mission_ctx = nullptr,
    ReturnMode return_mode = ReturnMode::RETURN_TO_HOME,
    double return_altitude_above_home_m = 15.0);

  ~ReturnStrategy() override = default;

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return completed_; }
  bool is_failed() const { return failed_; }
  StrategyType get_type() const override { return StrategyType::RETURN_STRATEGY; }
  std::string get_name() const override { return "RETURN_STRATEGY"; }

  ReturnMode get_return_mode() const { return return_mode_; }
  SubPhase get_sub_phase() const { return sub_phase_; }
  const std::string & failure_reason() const { return failure_reason_; }

private:
  void fail(const std::string & reason);

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> traj_setpoint_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;

  ReturnMode return_mode_{ReturnMode::RETURN_TO_HOME};
  SubPhase sub_phase_{SubPhase::APPROACH_HOME};

  double return_altitude_above_home_m_{15.0};
  double home_lat_{0.0};
  double home_lon_{0.0};
  double home_alt_msl_{0.0};
  bool home_initialized_{false};

  float dwell_timer_s_{0.0f};
  static constexpr float kTouchdownDwellDurationS = 0.5f;

  bool completed_{false};
  bool failed_{false};
  std::string failure_reason_;
};

}  // namespace full_self_driving::flight
