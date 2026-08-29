#include "flight/strategies/transit_out_strategy.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace full_self_driving::flight
{

TransitOutStrategy::TransitOutStrategy(
  rclcpp::Node & node,
  px4_ros2::Context & context,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  domain::Route route,
  std::shared_ptr<persistence::PersistenceManager> persistence,
  std::shared_ptr<domain::MissionContext> mission_ctx)
: node_(node),
  goto_setpoint_(std::make_shared<px4_ros2::GotoGlobalSetpointType>(context)),
  state_cache_(std::move(state_cache)),
  route_(std::move(route)),
  persistence_(std::move(persistence)),
  mission_ctx_(std::move(mission_ctx))
{
}

TransitOutStrategy::TransitOutStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  domain::Route route,
  std::shared_ptr<persistence::PersistenceManager> persistence,
  std::shared_ptr<domain::MissionContext> mission_ctx)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  state_cache_(std::move(state_cache)),
  route_(std::move(route)),
  persistence_(std::move(persistence)),
  mission_ctx_(std::move(mission_ctx))
{
}

void TransitOutStrategy::on_enter()
{
  waypoint_index_ = 0;
  target_altitude_msl_m_ = 0.0;
  target_altitude_set_ = false;
  mode_finished_ = false;
  failed_ = false;
  failure_reason_.clear();
  last_heading_valid_ = false;
  setpoint_sent_for_current_waypoint_ = false;
  activation_time_ = std::chrono::steady_clock::now();

  waypoints_ = route_.get_waypoints();
  if (waypoints_.empty()) {
    parameters_valid_ = true;
    mode_finished_ = true;
    RCLCPP_INFO(
      node_.get_logger(),
      "[TRANSIT_OUT] No Transit Out waypoints specified in plan. Skipping directly to Return RTL.");
    if (completion_cb_) {
      completion_cb_(true);
    }
    return;
  }

  std::vector<std::string> validation_errors;
  parameters_valid_ = route_.validate(&validation_errors);
  if (!parameters_valid_) {
    std::string err_str = "Invalid route parameters: ";
    for (const auto & e : validation_errors) {
      err_str += e + "; ";
    }
    fail(err_str);
    return;
  }

  transit_altitude_above_home_m_ = route_.get_transit_altitude_above_home_m();
  arrival_radius_m_ = route_.get_arrival_radius_m();
  max_horizontal_speed_m_s_ = route_.get_max_horizontal_speed_m_s();
  max_vertical_speed_m_s_ = route_.get_max_vertical_speed_m_s();
  max_heading_rate_rad_s_ = route_.get_max_heading_rate_rad_s();
  course_heading_min_speed_m_s_ = route_.get_course_heading_min_speed_m_s();
  altitude_tolerance_m_ = route_.get_altitude_tolerance_m();
  altitude_settle_speed_m_s_ = route_.get_altitude_settle_speed_m_s();
  data_timeout_s_ = route_.get_data_timeout_s();

  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
      last_heading_rad_ = snapshot.heading;
      last_heading_valid_ = true;
    }
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[TRANSIT_OUT] TransitOutStrategy entered with %zu waypoint(s), altitude %.2f m above home, arrival radius %.2f m, speed %.2f m/s",
    waypoints_.size(), transit_altitude_above_home_m_, arrival_radius_m_, max_horizontal_speed_m_s_);
}

void TransitOutStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[TRANSIT_OUT] TransitOutStrategy exited");
}

void TransitOutStrategy::on_update(float dt_s)
{
  (void)dt_s;

  if (mode_finished_ || failed_) {
    return;
  }

  if (waypoints_.empty()) {
    mode_finished_ = true;
    if (completion_cb_) {
      completion_cb_(true);
    }
    return;
  }

  if (!parameters_valid_) {
    fail("invalid transit parameters");
    return;
  }

  if (!state_cache_) {
    fail("PX4 state cache is missing");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();

  if (!snapshot.is_armed) {
    fail("vehicle is not armed; Transit Out requires an armed airborne vehicle");
    return;
  }

  if (!snapshot.land_detected_valid || snapshot.is_landed) {
    if (data_timed_out()) {
      fail("vehicle is landed or vehicle_land_detected data became stale during Transit Out");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[TRANSIT_OUT] Waiting for airborne state verification before starting Transit Out");
    }
    return;
  }

  if (!snapshot.home_pos_valid && (!mission_ctx_ || !mission_ctx_->has_origin_home_position())) {
    if (data_timed_out()) {
      fail("no valid PX4 home position was received");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[TRANSIT_OUT] Waiting for a valid PX4 home position before starting Transit Out");
    }
    return;
  }

  if (mission_ctx_ && mission_ctx_->has_origin_home_position()) {
    home_altitude_msl_m_ = mission_ctx_->get_origin_home_position().altitude_msl_m;
  } else {
    home_altitude_msl_m_ = snapshot.home_global_position.z();
  }

  if (!snapshot.global_pos_valid || !snapshot.local_pos_valid) {
    if (setpoint_sent_for_current_waypoint_ || data_timed_out()) {
      fail("required PX4 position data became invalid during Transit Out");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[TRANSIT_OUT] Waiting for valid PX4 global and local position data before starting Transit Out");
    }
    return;
  }

  if (!target_altitude_set_) {
    target_altitude_msl_m_ = home_altitude_msl_m_ + transit_altitude_above_home_m_;
    target_altitude_set_ = true;
    RCLCPP_INFO(
      node_.get_logger(), "[TRANSIT_OUT] Target altitude set to %.2f m AMSL (Origin Base: %.2f m, Rel: %.2f m)",
      target_altitude_msl_m_, home_altitude_msl_m_, transit_altitude_above_home_m_);
  }

  // Altitude gate: Climb in-place to transit altitude before advancing waypoints
  double current_alt_msl = snapshot.global_pos_valid ? snapshot.global_position.z() :
    (home_altitude_msl_m_ - snapshot.local_position_ned.z());
  if (current_alt_msl < target_altitude_msl_m_ - altitude_tolerance_m_) {
    double hold_lat = snapshot.global_pos_valid ? snapshot.global_position.x() :
      (snapshot.home_pos_valid ? snapshot.home_global_position.x() : 0.0);
    double hold_lon = snapshot.global_pos_valid ? snapshot.global_position.y() :
      (snapshot.home_pos_valid ? snapshot.home_global_position.y() : 0.0);
    const Eigen::Vector3d climb_target{hold_lat, hold_lon, target_altitude_msl_m_};
    const std::optional<float> heading = update_course_heading(snapshot);
    if (goto_setpoint_) {
      goto_setpoint_->update(
        climb_target, heading, max_horizontal_speed_m_s_, max_vertical_speed_m_s_,
        max_heading_rate_rad_s_);
    }
    return;
  }

  if (waypoint_index_ >= waypoints_.size()) {
    mode_finished_ = true;
    if (completion_cb_) {
      completion_cb_(true);
    }
    return;
  }

  const auto & wp = waypoints_[waypoint_index_];
  double target_alt = target_altitude_msl_m_;
  if (wp.altitude_m > 0.0 && wp.altitude_m != 10.0 && wp.altitude_m != 15.0 && wp.altitude_m != 20.0 && wp.altitude_m != transit_altitude_above_home_m_) {
    target_alt = home_altitude_msl_m_ + wp.altitude_m;
  }
  const Eigen::Vector3d target{wp.latitude_deg, wp.longitude_deg, target_alt};

  const std::optional<float> heading = update_course_heading(snapshot);
  if (goto_setpoint_) {
    goto_setpoint_->update(
      target, heading, max_horizontal_speed_m_s_, max_vertical_speed_m_s_,
      max_heading_rate_rad_s_);
  }

  if (!setpoint_sent_for_current_waypoint_) {
    setpoint_sent_for_current_waypoint_ = true;
    return;
  }

  if (!waypoint_reached(snapshot, target)) {
    return;
  }

  RCLCPP_INFO(
    node_.get_logger(), "[TRANSIT_OUT] Reached waypoint %zu/%zu: (lat=%.7f, lon=%.7f)",
    waypoint_index_ + 1, waypoints_.size(), wp.latitude_deg, wp.longitude_deg);

  if (waypoint_cb_) {
    waypoint_cb_(waypoint_index_, true);
  }

  // Checkpoint durable progression
  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = "EVT_TRANSIT_OUT_WAYPOINT_REACHED";
    entry.component = "transit_out_strategy";
    entry.detail = "Reached outbound waypoint " + std::to_string(waypoint_index_ + 1) + "/" + std::to_string(waypoints_.size());
    entry.timestamp_monotonic_ns = snapshot.monotonic_timestamp_ns;
    persistence_->append_journal_entry(entry);
  }

  ++waypoint_index_;
  setpoint_sent_for_current_waypoint_ = false;

  if (waypoint_index_ >= waypoints_.size()) {
    mode_finished_ = true;
    RCLCPP_INFO(node_.get_logger(), "[TRANSIT_OUT] All outbound waypoints completed successfully!");
    if (completion_cb_) {
      completion_cb_(true);
    }
  }
}

std::optional<float> TransitOutStrategy::update_course_heading(const adapters::Px4StateSnapshot & snapshot)
{
  if (snapshot.local_pos_valid) {
    const Eigen::Vector3f velocity = snapshot.local_velocity_ned;
    const float horizontal_speed = std::hypot(velocity.x(), velocity.y());
    if (std::isfinite(horizontal_speed) && horizontal_speed >= course_heading_min_speed_m_s_ &&
        std::isfinite(velocity.x()) && std::isfinite(velocity.y())) {
      last_heading_rad_ = std::atan2(velocity.y(), velocity.x());
      last_heading_valid_ = true;
    }
  }

  if (last_heading_valid_) {
    return last_heading_rad_;
  }

  if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
    last_heading_rad_ = snapshot.heading;
    last_heading_valid_ = true;
    return last_heading_rad_;
  }

  return std::nullopt;
}

bool TransitOutStrategy::waypoint_reached(
  const adapters::Px4StateSnapshot & snapshot,
  const Eigen::Vector3d & target) const
{
  const Eigen::Vector3d current_position = snapshot.global_position;
  const float horizontal_distance_m = px4_ros2::horizontalDistanceToGlobalPosition(
    current_position, target);
  const double altitude_error_m = std::abs(current_position.z() - target.z());

  if (!std::isfinite(horizontal_distance_m) || horizontal_distance_m > arrival_radius_m_ ||
      !std::isfinite(altitude_error_m) || altitude_error_m > altitude_tolerance_m_) {
    return false;
  }

  const float vertical_velocity_m_s = snapshot.local_velocity_ned.z();
  return std::isfinite(vertical_velocity_m_s) &&
         std::abs(vertical_velocity_m_s) <= altitude_settle_speed_m_s_;
}

bool TransitOutStrategy::data_timed_out() const
{
  if (state_cache_) {
    if (state_cache_->is_local_position_fresh(std::chrono::milliseconds(static_cast<int>(data_timeout_s_ * 1000.0f))) &&
        state_cache_->is_global_position_fresh(std::chrono::milliseconds(static_cast<int>(data_timeout_s_ * 1000.0f)))) {
      return false;
    }
  }

  if (activation_time_.time_since_epoch().count() == 0) {
    return false;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
    std::chrono::steady_clock::now() - activation_time_);
  return elapsed.count() > data_timeout_s_;
}

void TransitOutStrategy::fail(const std::string & reason)
{
  if (mode_finished_ || failed_) {
    return;
  }

  mode_finished_ = true;
  failed_ = true;
  failure_reason_ = reason;
  RCLCPP_ERROR(node_.get_logger(), "[TRANSIT_OUT] Transit Out strategy failed: %s", reason.c_str());
  if (completion_cb_) {
    completion_cb_(false);
  }
}

}  // namespace full_self_driving::flight
