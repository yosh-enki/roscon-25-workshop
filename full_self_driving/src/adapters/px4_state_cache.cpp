#include "adapters/px4_state_cache.hpp"
#include <cmath>

namespace full_self_driving::adapters
{

Px4StateCache::Px4StateCache(px4_ros2::Context & context)
: context_(context),
  local_pos_(context, false),
  global_pos_(context),
  home_pos_(context),
  land_detected_(context),
  vehicle_status_(context)
{
}

Px4StateSnapshot Px4StateCache::capture_snapshot() const
{
  Px4StateSnapshot snapshot;
  auto now = context_.node().get_clock()->now();
  snapshot.monotonic_timestamp_ns = now.nanoseconds();

  // Local Position
  if (local_pos_.lastValid(std::chrono::milliseconds(1000))) {
    snapshot.local_pos_valid = local_pos_.positionXYValid() && local_pos_.positionZValid();
    snapshot.local_position_ned = local_pos_.positionNed();
    if (local_pos_.velocityXYValid() && local_pos_.velocityZValid()) {
      snapshot.local_velocity_ned = local_pos_.velocityNed();
    }
    snapshot.local_acceleration_ned = local_pos_.accelerationNed();
    snapshot.heading = local_pos_.heading();
    snapshot.distance_ground = local_pos_.distanceGround();
  }

  // Global Position
  if (global_pos_.lastValid(std::chrono::milliseconds(1000))) {
    snapshot.global_pos_valid = global_pos_.positionValid();
    snapshot.global_position = global_pos_.position();
  }

  // Home Position (Latched message - cache once received)
  if (home_pos_.lastValid(std::chrono::hours(24))) {
    if (home_pos_.localPositionValid() || home_pos_.globaHorizontalPositionValid()) {
      home_received_ = true;
      cached_home_local_ = home_pos_.localPosition();
      cached_home_global_ = home_pos_.globalPosition();
      cached_home_yaw_ = home_pos_.yaw();
    }
  }
  if (home_received_) {
    snapshot.home_pos_valid = true;
    snapshot.home_local_position = cached_home_local_;
    snapshot.home_global_position = cached_home_global_;
    snapshot.home_yaw = cached_home_yaw_;
  }

  // Land Detected
  if (land_detected_.lastValid(std::chrono::milliseconds(1000))) {
    snapshot.land_detected_valid = true;
    snapshot.is_landed = land_detected_.landed();
  }

  // Vehicle Status
  if (vehicle_status_.lastValid(std::chrono::milliseconds(1000))) {
    snapshot.vehicle_status_valid = true;
    snapshot.is_armed = vehicle_status_.armed();
    snapshot.nav_state = vehicle_status_.navState();
  }

  return snapshot;
}

bool Px4StateCache::is_local_position_fresh(std::chrono::milliseconds timeout) const
{
  return local_pos_.lastValid(timeout) && local_pos_.positionXYValid() && local_pos_.positionZValid();
}

bool Px4StateCache::is_global_position_fresh(std::chrono::milliseconds timeout) const
{
  return global_pos_.lastValid(timeout) && global_pos_.positionValid();
}

bool Px4StateCache::is_home_position_fresh(std::chrono::milliseconds timeout) const
{
  (void)timeout;
  return home_received_ || (home_pos_.lastValid(std::chrono::hours(24)) &&
         (home_pos_.localPositionValid() || home_pos_.globaHorizontalPositionValid()));
}

bool Px4StateCache::is_land_detected_fresh(std::chrono::milliseconds timeout) const
{
  return land_detected_.lastValid(timeout);
}

bool Px4StateCache::is_vehicle_status_fresh(std::chrono::milliseconds timeout) const
{
  return vehicle_status_.lastValid(timeout);
}

bool Px4StateCache::is_transport_healthy() const
{
  return is_vehicle_status_fresh(std::chrono::milliseconds(2000));
}

bool Px4StateCache::is_armed() const
{
  if (vehicle_status_.lastValid()) {
    return vehicle_status_.armed();
  }
  return false;
}

bool Px4StateCache::is_landed() const
{
  if (land_detected_.lastValid()) {
    return land_detected_.landed();
  }
  return false;
}

uint8_t Px4StateCache::nav_state() const
{
  if (vehicle_status_.lastValid()) {
    return vehicle_status_.navState();
  }
  return 0;
}

float Px4StateCache::calculate_heading_from_velocity_or_attitude() const
{
  if (local_pos_.lastValid() && local_pos_.velocityXYValid()) {
    auto vel = local_pos_.velocityNed();
    float speed_sq = vel.x() * vel.x() + vel.y() * vel.y();
    if (speed_sq > 0.25f) {  // speed > 0.5 m/s
      return std::atan2(vel.y(), vel.x());
    }
  }
  if (local_pos_.lastValid()) {
    return local_pos_.heading();
  }
  return 0.f;
}

}  // namespace full_self_driving::adapters
