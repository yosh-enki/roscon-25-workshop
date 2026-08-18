#include "flight/strategies/takeoff_strategy.hpp"
#include <cmath>

namespace full_self_driving::flight
{

TakeoffStrategy::TakeoffStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  double target_altitude_above_home_m,
  double altitude_tolerance_m,
  double settle_speed_m_s,
  double timeout_s)
: node_(node),
  state_cache_(std::move(state_cache)),
  goto_setpoint_(std::move(goto_setpoint)),
  target_altitude_above_home_m_(target_altitude_above_home_m),
  altitude_tolerance_m_(altitude_tolerance_m),
  settle_speed_m_s_(settle_speed_m_s),
  timeout_s_(timeout_s)
{
}

void TakeoffStrategy::on_enter()
{
  start_time_ = std::chrono::steady_clock::now();
  completed_ = false;
  failed_ = false;
  failure_reason_.clear();
  settle_cycles_ = 0;
  target_altitude_set_ = false;

  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.home_pos_valid) {
      home_altitude_msl_m_ = snapshot.home_global_position.z();
      target_altitude_msl_m_ = home_altitude_msl_m_ + target_altitude_above_home_m_;
      target_altitude_set_ = true;
    }
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[TAKEOFF] TakeoffStrategy entered: target altitude %.2f m above home (AMSL: %.2f m)",
    target_altitude_above_home_m_, target_altitude_msl_m_);
}

void TakeoffStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[TAKEOFF] TakeoffStrategy exited");
}

void TakeoffStrategy::on_update(float dt_s)
{
  (void)dt_s;
  if (completed_ || failed_) {
    return;
  }

  if (!state_cache_) {
    fail("Px4StateCache is null");
    return;
  }

  // Check elapsed time
  auto now = std::chrono::steady_clock::now();
  double elapsed_s = std::chrono::duration<double>(now - start_time_).count();
  if (elapsed_s > timeout_s_) {
    fail("Takeoff timed out after " + std::to_string(elapsed_s) + " seconds");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();

  // Validate arming after a short grace period (1.0s)
  if (elapsed_s > 1.0 && !snapshot.is_armed) {
    fail("Vehicle is not armed during takeoff");
    return;
  }

  // Home position check
  if (!target_altitude_set_) {
    if (snapshot.home_pos_valid) {
      home_altitude_msl_m_ = snapshot.home_global_position.z();
      target_altitude_msl_m_ = home_altitude_msl_m_ + target_altitude_above_home_m_;
      target_altitude_set_ = true;
      RCLCPP_INFO(
        node_.get_logger(),
        "[TAKEOFF] Home position acquired: home alt %.2f m, target alt %.2f m AMSL",
        home_altitude_msl_m_, target_altitude_msl_m_);
    } else {
      if (elapsed_s > 5.0) {
        fail("No valid home position received during takeoff");
      }
      return;
    }
  }

  // Position and velocity checks
  if (!snapshot.local_pos_valid && !snapshot.global_pos_valid) {
    if (elapsed_s > 5.0) {
      fail("No valid position data received during takeoff");
    }
    return;
  }

  // Command climb setpoint to PX4
  if (target_altitude_set_ && goto_setpoint_) {
    double target_lat = snapshot.global_pos_valid ? snapshot.global_position.x() :
      (snapshot.home_pos_valid ? snapshot.home_global_position.x() : 0.0);
    double target_lon = snapshot.global_pos_valid ? snapshot.global_position.y() :
      (snapshot.home_pos_valid ? snapshot.home_global_position.y() : 0.0);
    Eigen::Vector3d target_pos{target_lat, target_lon, target_altitude_msl_m_};
    goto_setpoint_->update(target_pos, std::nullopt, 1.0f);
  }

  double current_alt_msl = snapshot.global_position.z();
  double height_above_home = current_alt_msl - home_altitude_msl_m_;
  if (!snapshot.global_pos_valid && snapshot.local_pos_valid) {
    height_above_home = -snapshot.local_position_ned.z();
  }

  double alt_error = std::abs(height_above_home - target_altitude_above_home_m_);
  float vertical_speed = std::abs(snapshot.local_velocity_ned.z());

  // Check height arrival and vertical speed settling
  if (alt_error <= altitude_tolerance_m_ && vertical_speed <= settle_speed_m_s_) {
    settle_cycles_++;
    if (settle_cycles_ >= kRequiredSettleCycles) {
      completed_ = true;
      RCLCPP_INFO(
        node_.get_logger(),
        "[TAKEOFF] Takeoff settled successfully at height %.2f m (target %.2f m, vz=%.2f m/s)",
        height_above_home, target_altitude_above_home_m_, vertical_speed);
      if (completion_cb_) {
        completion_cb_(true);
      }
    }
  } else {
    settle_cycles_ = 0;
  }
}

void TakeoffStrategy::fail(const std::string & reason)
{
  if (failed_ || completed_) {
    return;
  }
  failed_ = true;
  failure_reason_ = reason;
  RCLCPP_ERROR(node_.get_logger(), "[TAKEOFF] Takeoff failed: %s", reason.c_str());
  if (completion_cb_) {
    completion_cb_(false);
  }
}

}  // namespace full_self_driving::flight
