#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/geodesic.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "adapters/px4_state_cache.hpp"
#include "domain/live_target_lock.hpp"
#include "flight/internal_strategy.hpp"
#include "persistence/persistence_manager.hpp"

namespace full_self_driving::flight
{

enum class PrecisionLandSubPhase : uint8_t
{
  SEARCH = 0,
  HOVER_BRAKE = 1,     // Zero-velocity hover & attitude brake stabilization
  APPROACH = 2,        // Lateral centering and descent to approach altitude
  DESCEND = 3,         // Live P/PI-controlled lateral descent to touchdown
  LANDED_VERIFY = 4,   // Ground contact & stability dwell verification
  FINISHED = 5,
  FAILED = 6
};

inline const char * to_string(PrecisionLandSubPhase phase)
{
  switch (phase) {
    case PrecisionLandSubPhase::SEARCH: return "SEARCH";
    case PrecisionLandSubPhase::HOVER_BRAKE: return "HOVER_BRAKE";
    case PrecisionLandSubPhase::APPROACH: return "APPROACH";
    case PrecisionLandSubPhase::DESCEND: return "DESCEND";
    case PrecisionLandSubPhase::LANDED_VERIFY: return "LANDED_VERIFY";
    case PrecisionLandSubPhase::FINISHED: return "FINISHED";
    case PrecisionLandSubPhase::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

struct WorldTargetTag
{
  Eigen::Vector3d position{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
  };
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  uint64_t timestamp_ns{0};
  bool valid{false};
};

class PrecisionLandStrategy : public InternalStrategy
{
public:
  using CompletionCallback = std::function<void(bool /* success */)>;
  using SubPhaseCallback = std::function<void(PrecisionLandSubPhase)>;

  PrecisionLandStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    float max_velocity = 3.0f,           // STRICT PROTOTYPE PARITY: 3.0 m/s
    float descent_vel = 1.0f,            // STRICT PROTOTYPE PARITY: 1.0 m/s
    float vel_p_gain = 1.5f,             // STRICT PROTOTYPE PARITY: 1.5
    float vel_i_gain = 0.0f,             // STRICT PROTOTYPE PARITY: 0.0
    float target_timeout = 3.0f,         // STRICT PROTOTYPE PARITY: 3.0 s
    float delta_position = 0.25f,        // STRICT PROTOTYPE PARITY: 0.25 m
    float delta_velocity = 0.25f,        // STRICT PROTOTYPE PARITY: 0.25 m/s
    float stabilize_duration_s = 1.0f,   // Hover settle dwell duration: 1.0 s
    double search_altitude_m = 15.0,     // Search/cruise altitude AGL
    double approach_altitude_m = 5.0,    // Approach altitude AGL
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  PrecisionLandStrategy(
    rclcpp::Node & node,
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint,
    std::shared_ptr<adapters::Px4StateCache> state_cache,
    float max_velocity = 3.0f,           // STRICT PROTOTYPE PARITY: 3.0 m/s
    float descent_vel = 1.0f,            // STRICT PROTOTYPE PARITY: 1.0 m/s
    float vel_p_gain = 1.5f,             // STRICT PROTOTYPE PARITY: 1.5
    float vel_i_gain = 0.0f,             // STRICT PROTOTYPE PARITY: 0.0
    float target_timeout = 3.0f,         // STRICT PROTOTYPE PARITY: 3.0 s
    float delta_position = 0.25f,        // STRICT PROTOTYPE PARITY: 0.25 m
    float delta_velocity = 0.25f,        // STRICT PROTOTYPE PARITY: 0.25 m/s
    float stabilize_duration_s = 1.0f,   // Hover settle dwell duration: 1.0 s
    double search_altitude_m = 15.0,     // Search/cruise altitude AGL
    double approach_altitude_m = 5.0,    // Approach altitude AGL
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);

  ~PrecisionLandStrategy() override = default;

  void set_completion_callback(CompletionCallback cb) { completion_cb_ = std::move(cb); }
  void set_subphase_callback(SubPhaseCallback cb) { subphase_cb_ = std::move(cb); }

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return mode_finished_ && !failed_; }
  bool is_failed() const { return failed_; }
  StrategyType get_type() const override { return StrategyType::PRECISION_LAND; }
  std::string get_name() const override { return "PRECISION_LAND"; }

  // Target lock update interface
  void update_target_lock(const domain::LiveTargetLock & lock);

  // Sub-phase and state inspection
  PrecisionLandSubPhase get_sub_phase() const { return sub_phase_; }
  std::string get_sub_phase_name() const { return to_string(sub_phase_); }
  bool is_hover_stabilized() const { return hover_stabilized_; }
  float hover_settle_duration() const { return hover_settle_duration_s_; }
  float current_velocity_norm() const { return current_velocity_norm_; }
  const std::string & failure_reason() const { return failure_reason_; }

  // Prototype parameter accessors (for strict parity verification)
  float max_velocity() const { return max_velocity_; }
  float descent_vel() const { return descent_vel_; }
  float vel_p_gain() const { return vel_p_gain_; }
  float vel_i_gain() const { return vel_i_gain_; }
  float target_timeout() const { return target_timeout_; }
  float delta_position() const { return delta_position_; }
  float delta_velocity() const { return delta_velocity_; }
  float stabilize_duration() const { return stabilize_duration_s_; }
  double search_altitude_m() const { return search_altitude_m_; }
  double approach_altitude_m() const { return approach_altitude_m_; }

  // Transform helper (Optical -> Drone Body FRD -> World NED)
  static WorldTargetTag compute_world_tag(
    const geometry_msgs::msg::Pose & tag_pose_optical,
    const adapters::Px4StateSnapshot & snapshot);

  // Lateral velocity P/PI controller calculation (Prototype Parity with discrete dt_s integration)
  Eigen::Vector2f calculate_velocity_setpoint_xy(
    const Eigen::Vector3d & vehicle_pos_ned,
    const Eigen::Vector3d & tag_pos_ned,
    float dt_s = 0.02f);

  // Spiral search waypoint generation (Prototype Parity)
  std::vector<Eigen::Vector3f> generate_search_waypoints(const Eigen::Vector3f & start_pos_ned);
  const std::vector<Eigen::Vector3f> & search_waypoints() const { return search_waypoints_; }

private:
  void switch_to_sub_phase(PrecisionLandSubPhase next_phase);
  void fail(const std::string & reason);
  bool check_target_timeout(uint64_t now_ns);

  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint_;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;

  // Prototype-Parity Configuration Parameters
  float max_velocity_{3.0f};
  float descent_vel_{1.0f};
  float vel_p_gain_{1.5f};
  float vel_i_gain_{0.0f};
  float target_timeout_{3.0f};
  float delta_position_{0.25f};
  float delta_velocity_{0.25f};
  float stabilize_duration_s_{1.0f};
  double search_altitude_m_{15.0};
  double approach_altitude_m_{5.0};

  // Internal State
  PrecisionLandSubPhase sub_phase_{PrecisionLandSubPhase::SEARCH};
  bool mode_finished_{false};
  bool failed_{false};
  std::string failure_reason_;

  // Altitude & GPS references
  double home_altitude_msl_m_{0.0};
  double search_altitude_amsl_m_{15.0};
  double approach_altitude_amsl_m_{5.0};
  double descend_altitude_amsl_m_{5.0};
  bool altitude_references_set_{false};
  float landed_dwell_s_{0.0f};

  // Brake / Hover Stabilization State
  Eigen::Vector3d brake_hold_global_{0.0, 0.0, 0.0};
  Eigen::Vector3f brake_hold_local_ned_{0.0f, 0.0f, 0.0f};
  bool brake_hold_position_captured_{false};
  bool brake_position_locked_{false};
  float hover_settle_duration_s_{0.0f};
  bool hover_stabilized_{false};
  float current_velocity_norm_{0.0f};

  // Target Tracking Data
  WorldTargetTag last_world_tag_;
  domain::LiveTargetLock last_lock_;
  bool target_acquired_{false};
  uint64_t last_lock_received_ns_{0};

  // Lateral Controller Integrators
  float vel_x_integral_{0.0f};
  float vel_y_integral_{0.0f};

  // Spiral search waypoints
  std::vector<Eigen::Vector3f> search_waypoints_;
  size_t search_waypoint_index_{0};

  // Callbacks
  CompletionCallback completion_cb_;
  SubPhaseCallback subphase_cb_;
};

}  // namespace full_self_driving::flight
