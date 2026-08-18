#include "flight/strategies/direct_strategy.hpp"
#include <cmath>

namespace full_self_driving::flight
{

DirectStrategy::DirectStrategy(
  rclcpp::Node & node,
  px4_ros2::Context & context,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  double target_latitude_deg,
  double target_longitude_deg,
  double target_altitude_above_home_m,
  float arrival_radius_m,
  float settle_speed_m_s,
  float settle_duration_s,
  float max_horizontal_speed_m_s,
  float max_yaw_rate_rad_s,
  double direct_timeout_s,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  target_latitude_deg_(target_latitude_deg),
  target_longitude_deg_(target_longitude_deg),
  target_altitude_above_home_m_(target_altitude_above_home_m),
  arrival_radius_m_(arrival_radius_m),
  settle_speed_m_s_(settle_speed_m_s),
  settle_duration_s_(settle_duration_s),
  max_horizontal_speed_m_s_(max_horizontal_speed_m_s),
  max_yaw_rate_rad_s_(max_yaw_rate_rad_s),
  direct_timeout_s_(direct_timeout_s)
{
  (void)context;
  if (!std::isfinite(target_latitude_deg_) || target_latitude_deg_ < -90.0 || target_latitude_deg_ > 90.0 ||
      !std::isfinite(target_longitude_deg_) || target_longitude_deg_ < -180.0 || target_longitude_deg_ > 180.0 ||
      !std::isfinite(target_altitude_above_home_m_) || target_altitude_above_home_m_ < 0.0 ||
      arrival_radius_m_ <= 0.0f || settle_speed_m_s_ <= 0.0f || settle_duration_s_ < 0.0f ||
      max_horizontal_speed_m_s_ <= 0.0f || max_yaw_rate_rad_s_ <= 0.0f || direct_timeout_s_ <= 0.0)
  {
    parameters_valid_ = false;
  }
}

DirectStrategy::DirectStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  double target_latitude_deg,
  double target_longitude_deg,
  double target_altitude_above_home_m,
  float arrival_radius_m,
  float settle_speed_m_s,
  float settle_duration_s,
  float max_horizontal_speed_m_s,
  float max_yaw_rate_rad_s,
  double direct_timeout_s,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  target_latitude_deg_(target_latitude_deg),
  target_longitude_deg_(target_longitude_deg),
  target_altitude_above_home_m_(target_altitude_above_home_m),
  arrival_radius_m_(arrival_radius_m),
  settle_speed_m_s_(settle_speed_m_s),
  settle_duration_s_(settle_duration_s),
  max_horizontal_speed_m_s_(max_horizontal_speed_m_s),
  max_yaw_rate_rad_s_(max_yaw_rate_rad_s),
  direct_timeout_s_(direct_timeout_s)
{
  if (!std::isfinite(target_latitude_deg_) || target_latitude_deg_ < -90.0 || target_latitude_deg_ > 90.0 ||
      !std::isfinite(target_longitude_deg_) || target_longitude_deg_ < -180.0 || target_longitude_deg_ > 180.0 ||
      !std::isfinite(target_altitude_above_home_m_) || target_altitude_above_home_m_ < 0.0 ||
      arrival_radius_m_ <= 0.0f || settle_speed_m_s_ <= 0.0f || settle_duration_s_ < 0.0f ||
      max_horizontal_speed_m_s_ <= 0.0f || max_yaw_rate_rad_s_ <= 0.0f || direct_timeout_s_ <= 0.0)
  {
    parameters_valid_ = false;
  }
}

DirectStrategy::DirectStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  const full_self_driving::msg::PadRecord & pad_record,
  double target_altitude_above_home_m,
  float arrival_radius_m,
  float settle_speed_m_s,
  float settle_duration_s,
  float max_horizontal_speed_m_s,
  float max_yaw_rate_rad_s,
  double direct_timeout_s,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: DirectStrategy(
    node,
    std::move(goto_setpoint),
    std::move(state_cache),
    pad_record.latitude_deg,
    pad_record.longitude_deg,
    target_altitude_above_home_m,
    arrival_radius_m,
    settle_speed_m_s,
    settle_duration_s,
    max_horizontal_speed_m_s,
    max_yaw_rate_rad_s,
    direct_timeout_s,
    std::move(persistence))
{
}

void DirectStrategy::on_enter()
{
  mode_finished_ = false;
  failed_ = false;
  settled_ = false;
  failure_reason_.clear();
  target_altitude_set_ = false;
  setpoint_sent_ = false;
  settle_accumulated_s_ = 0.0f;
  activation_time_ = std::chrono::steady_clock::now();

  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
      last_heading_rad_ = snapshot.heading;
      last_heading_valid_ = true;
    }
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[DIRECT] Direct navigation activated for pad (lat=%.6f, lon=%.6f, alt_above_home=%.2f m, arrival_rad=%.2f m, timeout=%.1f s)",
    target_latitude_deg_, target_longitude_deg_, target_altitude_above_home_m_, arrival_radius_m_, direct_timeout_s_);
}

void DirectStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[DIRECT] Direct navigation deactivated");
}

void DirectStrategy::on_update(float dt_s)
{
  if (mode_finished_ || failed_) {
    return;
  }

  if (!parameters_valid_) {
    fail("invalid direct navigation parameters");
    return;
  }

  if (!state_cache_) {
    fail("Px4StateCache is null");
    return;
  }

  // Check timeout
  auto elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - activation_time_).count();
  if (elapsed_s > direct_timeout_s_) {
    fail("direct navigation timed out after " + std::to_string(elapsed_s) + " s");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();

  if (!snapshot.is_armed) {
    fail("vehicle is not armed; Direct navigation requires an airborne vehicle");
    return;
  }

  if (!snapshot.land_detected_valid) {
    if (data_timed_out()) {
      fail("no fresh vehicle_land_detected sample received");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[DIRECT] Waiting for a fresh PX4 vehicle_land_detected sample");
    }
    return;
  }

  if (snapshot.is_landed) {
    fail("vehicle is landed; Direct navigation must be executed while airborne");
    return;
  }

  if (!snapshot.home_pos_valid) {
    if (data_timed_out()) {
      fail("no valid PX4 home position was received");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[DIRECT] Waiting for a valid PX4 home position");
    }
    return;
  }
  home_altitude_msl_m_ = snapshot.home_global_position.z();

  if (!snapshot.global_pos_valid || !snapshot.local_pos_valid) {
    if (data_timed_out()) {
      fail("required PX4 position data became invalid during Direct navigation");
    } else {
      RCLCPP_INFO_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "[DIRECT] Waiting for valid PX4 global and local position data");
    }
    return;
  }

  if (!target_altitude_set_) {
    target_altitude_amsl_m_ = home_altitude_msl_m_ + target_altitude_above_home_m_;
    target_altitude_set_ = true;
    RCLCPP_INFO(
      node_.get_logger(),
      "[DIRECT] Direct target altitude is %.2f m AMSL (home=%.2f m AMSL + relative=%.2f m)",
      target_altitude_amsl_m_, home_altitude_msl_m_, target_altitude_above_home_m_);
  }

  const Eigen::Vector3d target{target_latitude_deg_, target_longitude_deg_, target_altitude_amsl_m_};
  const std::optional<float> heading = update_course_heading(snapshot);

  if (goto_setpoint_) {
    goto_setpoint_->update(
      target, heading, max_horizontal_speed_m_s_, max_vertical_speed_m_s_,
      max_yaw_rate_rad_s_);
  }

  if (!setpoint_sent_) {
    setpoint_sent_ = true;
    return;
  }

  // Evaluate arrival and velocity settle gates
  const Eigen::Vector3d current_position = snapshot.global_position;
  const float horizontal_distance_m = px4_ros2::horizontalDistanceToGlobalPosition(current_position, target);
  const float horizontal_speed = std::hypot(snapshot.local_velocity_ned.x(), snapshot.local_velocity_ned.y());
  const float vertical_speed = std::abs(snapshot.local_velocity_ned.z());
  const double altitude_error_m = std::abs(current_position.z() - target_altitude_amsl_m_);

  if (std::isfinite(horizontal_distance_m) && horizontal_distance_m <= arrival_radius_m_ &&
      std::isfinite(horizontal_speed) && horizontal_speed <= settle_speed_m_s_ &&
      std::isfinite(vertical_speed) && vertical_speed <= settle_speed_m_s_ &&
      std::isfinite(altitude_error_m) && altitude_error_m <= altitude_tolerance_m_)
  {
    settle_accumulated_s_ += dt_s;
    if (settle_accumulated_s_ >= settle_duration_s_) {
      settled_ = true;
      mode_finished_ = true;
      RCLCPP_INFO(
        node_.get_logger(),
        "[DIRECT] Direct navigation completed and settled at safe altitude above pad (lat=%.6f, lon=%.6f, alt=%.2f m AMSL, dist=%.2f m)",
        target_latitude_deg_, target_longitude_deg_, target_altitude_amsl_m_, horizontal_distance_m);

      if (persistence_) {
        persistence::JournalEntry entry;
        entry.event_id = "EVT_DIRECT_COMPLETE";
        entry.component = "DirectStrategy";
        entry.detail = "Direct navigation completed and settled at safe altitude above pad";
        entry.timestamp_monotonic_ns = this->node_.get_clock()->now().nanoseconds();
        persistence_->append_journal_entry(entry);
      }

      if (completion_cb_) {
        completion_cb_(true);
      }
    }
  } else {
    settle_accumulated_s_ = 0.0f;
  }
}

void DirectStrategy::fail(const std::string & reason)
{
  if (failed_) {
    return;
  }
  failed_ = true;
  failure_reason_ = reason;
  RCLCPP_ERROR(node_.get_logger(), "[DIRECT] Direct navigation failed: %s", reason.c_str());

  if (completion_cb_) {
    completion_cb_(false);
  }
}

bool DirectStrategy::data_timed_out() const
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

  auto elapsed_s = std::chrono::duration<float>(std::chrono::steady_clock::now() - activation_time_).count();
  return elapsed_s > data_timeout_s_;
}

std::optional<float> DirectStrategy::update_course_heading(const adapters::Px4StateSnapshot & snapshot)
{
  if (snapshot.local_pos_valid) {
    const float speed_xy = std::hypot(snapshot.local_velocity_ned.x(), snapshot.local_velocity_ned.y());
    if (std::isfinite(speed_xy) && speed_xy >= 0.3f &&
        std::isfinite(snapshot.local_velocity_ned.x()) && std::isfinite(snapshot.local_velocity_ned.y())) {
      const float course = std::atan2(snapshot.local_velocity_ned.y(), snapshot.local_velocity_ned.x());
      last_heading_rad_ = course;
      last_heading_valid_ = true;
      return course;
    }
  }

  if (last_heading_valid_) {
    return last_heading_rad_;
  }

  if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
    last_heading_rad_ = snapshot.heading;
    last_heading_valid_ = true;
    return snapshot.heading;
  }

  return std::nullopt;
}

}  // namespace full_self_driving::flight
