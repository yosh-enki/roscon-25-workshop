#include "flight/strategies/precision_land_strategy.hpp"
#include <algorithm>
#include <cmath>

namespace full_self_driving::flight
{

PrecisionLandStrategy::PrecisionLandStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  float max_velocity,
  float descent_vel,
  float vel_p_gain,
  float vel_i_gain,
  float target_timeout,
  float delta_position,
  float delta_velocity,
  float stabilize_duration_s,
  double search_altitude_m,
  double approach_altitude_m,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  max_velocity_(max_velocity),
  descent_vel_(descent_vel),
  vel_p_gain_(vel_p_gain),
  vel_i_gain_(vel_i_gain),
  target_timeout_(target_timeout),
  delta_position_(delta_position),
  delta_velocity_(delta_velocity),
  stabilize_duration_s_(stabilize_duration_s),
  search_altitude_m_(search_altitude_m),
  approach_altitude_m_(approach_altitude_m)
{
  if (!std::isfinite(max_velocity_) || max_velocity_ <= 0.0f ||
      !std::isfinite(descent_vel_) || descent_vel_ <= 0.0f ||
      !std::isfinite(vel_p_gain_) || vel_p_gain_ <= 0.0f ||
      !std::isfinite(target_timeout_) || target_timeout_ <= 0.0f ||
      !std::isfinite(delta_position_) || delta_position_ <= 0.0f ||
      !std::isfinite(delta_velocity_) || delta_velocity_ <= 0.0f ||
      !std::isfinite(stabilize_duration_s_) || stabilize_duration_s_ <= 0.0f)
  {
    fail("PrecisionLandStrategy parameters must be finite and positive");
  }
}

PrecisionLandStrategy::PrecisionLandStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  float max_velocity,
  float descent_vel,
  float vel_p_gain,
  float vel_i_gain,
  float target_timeout,
  float delta_position,
  float delta_velocity,
  float stabilize_duration_s,
  double search_altitude_m,
  double approach_altitude_m,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  trajectory_setpoint_(std::move(trajectory_setpoint)),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  max_velocity_(max_velocity),
  descent_vel_(descent_vel),
  vel_p_gain_(vel_p_gain),
  vel_i_gain_(vel_i_gain),
  target_timeout_(target_timeout),
  delta_position_(delta_position),
  delta_velocity_(delta_velocity),
  stabilize_duration_s_(stabilize_duration_s),
  search_altitude_m_(search_altitude_m),
  approach_altitude_m_(approach_altitude_m)
{
  if (!std::isfinite(max_velocity_) || max_velocity_ <= 0.0f ||
      !std::isfinite(descent_vel_) || descent_vel_ <= 0.0f ||
      !std::isfinite(vel_p_gain_) || vel_p_gain_ <= 0.0f ||
      !std::isfinite(target_timeout_) || target_timeout_ <= 0.0f ||
      !std::isfinite(delta_position_) || delta_position_ <= 0.0f ||
      !std::isfinite(delta_velocity_) || delta_velocity_ <= 0.0f ||
      !std::isfinite(stabilize_duration_s_) || stabilize_duration_s_ <= 0.0f)
  {
    fail("PrecisionLandStrategy parameters must be finite and positive");
  }
}

void PrecisionLandStrategy::fail(const std::string & reason)
{
  failed_ = true;
  failure_reason_ = reason;
  sub_phase_ = PrecisionLandSubPhase::FAILED;
  RCLCPP_ERROR(node_.get_logger(), "[PRECISION_LAND] Strategy failed: %s", reason.c_str());

  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = "PRECISION_LAND_FAILED";
    entry.detail = reason;
    persistence_->append_journal_entry(entry);
  }

  if (completion_cb_) {
    completion_cb_(false);
  }
}

void PrecisionLandStrategy::switch_to_sub_phase(PrecisionLandSubPhase next_phase)
{
  if (sub_phase_ == next_phase) {
    return;
  }
  RCLCPP_INFO(
    node_.get_logger(),
    "[PRECISION_LAND] Sub-phase transition: %s -> %s",
    to_string(sub_phase_), to_string(next_phase));

  sub_phase_ = next_phase;

  if (subphase_cb_) {
    subphase_cb_(sub_phase_);
  }

  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = std::string("PRECISION_LAND_") + to_string(next_phase);
    entry.detail = "Sub-phase changed to " + std::string(to_string(next_phase));
    persistence_->append_journal_entry(entry);
  }
}

void PrecisionLandStrategy::on_enter()
{
  failed_ = false;
  mode_finished_ = false;
  failure_reason_.clear();
  hover_stabilized_ = false;
  hover_settle_duration_s_ = 0.0f;
  brake_hold_position_captured_ = false;
  target_acquired_ = false;
  vel_x_integral_ = 0.0f;
  vel_y_integral_ = 0.0f;

  if (!state_cache_ || !goto_setpoint_) {
    fail("State cache or Goto setpoint type is uninitialized");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();
  if (snapshot.home_pos_valid) {
    home_altitude_msl_m_ = snapshot.home_global_position.z();
    search_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
    approach_altitude_amsl_m_ = home_altitude_msl_m_ + approach_altitude_m_;
    altitude_references_set_ = true;
  } else if (snapshot.global_pos_valid) {
    home_altitude_msl_m_ = snapshot.global_position.z();
    search_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
    approach_altitude_amsl_m_ = home_altitude_msl_m_ + approach_altitude_m_;
    altitude_references_set_ = true;
  }

  // Generate spiral search waypoints from initial local NED position
  if (snapshot.local_pos_valid) {
    generate_search_waypoints(snapshot.local_position_ned);
  }

  switch_to_sub_phase(PrecisionLandSubPhase::SEARCH);

  RCLCPP_INFO(
    node_.get_logger(),
    "[PRECISION_LAND] PrecisionLandStrategy entered. Sub-phase: SEARCH at altitude %.2f m AMSL (max_vel: %.1f m/s, p_gain: %.2f)",
    search_altitude_amsl_m_, max_velocity_, vel_p_gain_);
}

void PrecisionLandStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[PRECISION_LAND] PrecisionLandStrategy exited.");
}

void PrecisionLandStrategy::update_target_lock(const domain::LiveTargetLock & lock)
{
  if (failed_ || mode_finished_) {
    return;
  }

  last_lock_ = lock;
  last_lock_received_ns_ = lock.received_monotonic_ns;

  if (lock.is_qualified()) {
    if (state_cache_) {
      auto snapshot = state_cache_->capture_snapshot();
      WorldTargetTag new_tag = compute_world_tag(lock.pose, snapshot);

      // Low-pass EMA filter (alpha = 0.75) to smooth out pixel discretization jitter at 15m
      if (last_world_tag_.valid && target_acquired_) {
        const double alpha = 0.75;
        new_tag.position = alpha * new_tag.position + (1.0 - alpha) * last_world_tag_.position;
        new_tag.orientation = last_world_tag_.orientation.slerp(alpha, new_tag.orientation);
      }

      last_world_tag_ = new_tag;
      target_acquired_ = true;

      // When target is acquired during SEARCH, initiate the zero-velocity hover brake phase
      if (sub_phase_ == PrecisionLandSubPhase::SEARCH) {
        brake_hold_global_ = snapshot.global_position;
        brake_hold_local_ned_ = snapshot.local_position_ned;
        brake_hold_position_captured_ = false;
        brake_position_locked_ = false;
        hover_settle_duration_s_ = 0.0f;
        hover_stabilized_ = false;

        RCLCPP_INFO(
          node_.get_logger(),
          "[PRECISION_LAND] Target acquired! Initiating coasting brake to zero-velocity hover (no reversing)...");

        switch_to_sub_phase(PrecisionLandSubPhase::HOVER_BRAKE);
      }
    }
  }
}

void PrecisionLandStrategy::on_update(float dt_s)
{
  if (failed_ || mode_finished_) {
    return;
  }

  if (!state_cache_ || !goto_setpoint_) {
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();
  if (!snapshot.global_pos_valid || !snapshot.local_pos_valid) {
    return;
  }

  if (!altitude_references_set_ && snapshot.home_pos_valid) {
    home_altitude_msl_m_ = snapshot.home_global_position.z();
    search_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
    approach_altitude_amsl_m_ = home_altitude_msl_m_ + approach_altitude_m_;
    altitude_references_set_ = true;
  }

  // Calculate current velocity norm
  float vx = snapshot.local_velocity_ned.x();
  float vy = snapshot.local_velocity_ned.y();
  float vz = snapshot.local_velocity_ned.z();
  current_velocity_norm_ = std::sqrt(vx * vx + vy * vy + vz * vz);
  float h_speed = std::sqrt(vx * vx + vy * vy);

  // Check target timeout
  bool target_lost = check_target_timeout(snapshot.monotonic_timestamp_ns);

  // State Machine Execution
  switch (sub_phase_) {
    case PrecisionLandSubPhase::SEARCH: {
      // In SEARCH, maintain search altitude and hold position or follow spiral waypoints
      Eigen::Vector3d hold_target(
        snapshot.global_position.x(),
        snapshot.global_position.y(),
        search_altitude_amsl_m_);
      goto_setpoint_->update(hold_target, std::nullopt, max_velocity_);
      break;
    }

    case PrecisionLandSubPhase::HOVER_BRAKE: {
      // NATURAL COAST-TO-STOP & HOVER GATE:
      // While decelerating, continuously advance the brake target to the drone's instantaneous position.
      // This prevents PX4 from generating a backwards position error and reversing.
      if (!brake_position_locked_) {
        brake_hold_global_ = snapshot.global_position;
        brake_hold_local_ned_ = snapshot.local_position_ned;
      }

      Eigen::Vector3d brake_target(
        brake_hold_global_.x(),
        brake_hold_global_.y(),
        search_altitude_amsl_m_);

      // Command position hold / zero velocity damping
      goto_setpoint_->update(brake_target, std::nullopt, max_velocity_);

      // Check if vehicle velocity is settled below delta_velocity threshold
      bool velocity_settled = (h_speed <= delta_velocity_) && (std::abs(vz) <= delta_velocity_);

      if (velocity_settled) {
        hover_settle_duration_s_ += dt_s;
        if (!brake_position_locked_) {
          // Lock hover coordinate precisely where the vehicle came to rest!
          brake_position_locked_ = true;
          RCLCPP_INFO(
            node_.get_logger(),
            "[PRECISION_LAND] Forward momentum neutralized. Locking hover position at (lat=%.6f, lon=%.6f, alt=%.2f m)",
            brake_hold_global_.x(), brake_hold_global_.y(), search_altitude_amsl_m_);
        }

        if (hover_settle_duration_s_ >= stabilize_duration_s_ && !hover_stabilized_) {
          hover_stabilized_ = true;
          RCLCPP_INFO(
            node_.get_logger(),
            "[PRECISION_LAND] Zero-velocity hover STABILIZED! Drone is stationary "
            "(vel=%.3f m/s, settle_time=%.2fs). Auto-triggering Prototype Precision Land sequence (APPROACH)...",
            current_velocity_norm_, hover_settle_duration_s_);
          switch_to_sub_phase(PrecisionLandSubPhase::APPROACH);
        }
      } else {
        // Reset dwell timer if velocity spikes or motion occurs
        hover_settle_duration_s_ = 0.0f;
        hover_stabilized_ = false;
        brake_position_locked_ = false;
      }

      // If target is lost during hover brake, handle target timeout
      if (target_lost) {
        RCLCPP_WARN(
          node_.get_logger(),
          "[PRECISION_LAND] Target lock lost during HOVER_BRAKE (timeout: %.1fs)",
          target_timeout_);
      }

      break;
    }

    case PrecisionLandSubPhase::APPROACH: {
      if (target_lost) {
        RCLCPP_WARN(node_.get_logger(), "[PRECISION_LAND] Target lost during APPROACH!");
        fail("Target lost during APPROACH");
        return;
      }

      // Approach using position setpoints over landing target while maintaining approach altitude
      Eigen::Vector3f target_position_ned(
        static_cast<float>(last_world_tag_.position.x()),
        static_cast<float>(last_world_tag_.position.y()),
        static_cast<float>(-approach_altitude_m_));

      if (trajectory_setpoint_) {
        // Prototype Parity: position setpoint in local NED
        trajectory_setpoint_->updatePosition(target_position_ned);
      } else if (goto_setpoint_) {
        double home_lat = snapshot.home_global_position.x();
        double home_lon = snapshot.home_global_position.y();
        double lat_rad = home_lat * M_PI / 180.0;
        double target_lat = home_lat + (last_world_tag_.position.x() / 111132.954);
        double target_lon = home_lon + (last_world_tag_.position.y() / (111132.954 * std::cos(lat_rad)));
        Eigen::Vector3d target_global(target_lat, target_lon, approach_altitude_amsl_m_);

        double siny_cosp = 2.0 * (last_world_tag_.orientation.w() * last_world_tag_.orientation.z() +
                                  last_world_tag_.orientation.x() * last_world_tag_.orientation.y());
        double cosy_cosp = 1.0 - 2.0 * (last_world_tag_.orientation.y() * last_world_tag_.orientation.y() +
                                        last_world_tag_.orientation.z() * last_world_tag_.orientation.z());
        float tag_yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
        goto_setpoint_->update(target_global, tag_yaw, max_velocity_);
      }

      // Check positionReached (Prototype Parity): (delta_pos.norm() < delta_position_) && (velocity.norm() < delta_velocity_)
      Eigen::Vector3f delta_pos = target_position_ned - snapshot.local_position_ned;
      float delta_pos_norm = delta_pos.norm();

      if (delta_pos_norm < delta_position_ && current_velocity_norm_ < delta_velocity_) {
        RCLCPP_INFO(
          node_.get_logger(),
          "[PRECISION_LAND] Target centered at approach altitude! Switching to DESCEND (pos_err=%.3fm, vel=%.3fm/s)",
          delta_pos_norm, current_velocity_norm_);
        switch_to_sub_phase(PrecisionLandSubPhase::DESCEND);
      }
      break;
    }

    case PrecisionLandSubPhase::DESCEND: {
      if (target_lost) {
        RCLCPP_WARN(node_.get_logger(), "[PRECISION_LAND] Target lost during DESCEND!");
        fail("Target lost during DESCEND");
        return;
      }

      // Calculate lateral velocity using exact Prototype P-controller with discrete dt_s integration
      Eigen::Vector2f vel_sp = calculate_velocity_setpoint_xy(
        snapshot.local_position_ned.cast<double>(),
        last_world_tag_.position,
        dt_s);

      double siny_cosp = 2.0 * (last_world_tag_.orientation.w() * last_world_tag_.orientation.z() +
                                last_world_tag_.orientation.x() * last_world_tag_.orientation.y());
      double cosy_cosp = 1.0 - 2.0 * (last_world_tag_.orientation.y() * last_world_tag_.orientation.y() +
                                      last_world_tag_.orientation.z() * last_world_tag_.orientation.z());
      float tag_yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

      if (trajectory_setpoint_) {
        // EXACT PROTOTYPE PARITY (Lines 190-192): send 3D Velocity Vector (vx, vy, descent_vel) directly to PX4
        trajectory_setpoint_->update(
          Eigen::Vector3f(vel_sp.x(), vel_sp.y(), descent_vel_),
          std::nullopt,
          tag_yaw);
      } else if (goto_setpoint_) {
        double home_lat = snapshot.home_global_position.x();
        double home_lon = snapshot.home_global_position.y();
        double lat_rad = home_lat * M_PI / 180.0;
        double target_lat = home_lat + (last_world_tag_.position.x() / 111132.954);
        double target_lon = home_lon + (last_world_tag_.position.y() / (111132.954 * std::cos(lat_rad)));
        Eigen::Vector3d target_global(target_lat, target_lon, home_altitude_msl_m_);
        goto_setpoint_->update(target_global, tag_yaw, max_velocity_);
      }

      // Check touchdown / land detection
      if (snapshot.is_landed || (snapshot.local_position_ned.z() >= -0.3f && std::abs(vz) < 0.25f)) {
        RCLCPP_INFO(node_.get_logger(), "[PRECISION_LAND] Touchdown detected! Verifying landing...");
        landed_dwell_s_ = 0.0f;
        switch_to_sub_phase(PrecisionLandSubPhase::LANDED_VERIFY);
      }
      break;
    }

    case PrecisionLandSubPhase::LANDED_VERIFY: {
      if (trajectory_setpoint_) {
        trajectory_setpoint_->update(
          Eigen::Vector3f(0.0f, 0.0f, 0.0f),
          std::nullopt,
          std::nullopt);
      }
      landed_dwell_s_ += dt_s;
      if (landed_dwell_s_ >= 0.5f) {
        RCLCPP_INFO(node_.get_logger(), "[PRECISION_LAND] Landing verified and completed successfully!");
        switch_to_sub_phase(PrecisionLandSubPhase::FINISHED);
        mode_finished_ = true;
        if (completion_cb_) {
          completion_cb_(true);
        }
      }
      break;
    }

    case PrecisionLandSubPhase::FINISHED:
    case PrecisionLandSubPhase::FAILED:
      break;
  }
}

bool PrecisionLandStrategy::check_target_timeout(uint64_t now_ns)
{
  if (!last_world_tag_.valid || last_lock_received_ns_ == 0) {
    return true;
  }
  if (now_ns >= last_lock_received_ns_) {
    double age_s = static_cast<double>(now_ns - last_lock_received_ns_) * 1e-9;
    return age_s > target_timeout_;
  }
  return false;
}

WorldTargetTag PrecisionLandStrategy::compute_world_tag(
  const geometry_msgs::msg::Pose & tag_pose_optical,
  const adapters::Px4StateSnapshot & snapshot)
{
  // Optical to FRD camera mounting rotation:
  // Optical: X right, Y down, Z forward
  // FRD:     X forward, Y right, Z down
  Eigen::Matrix3d R_cam_to_body;
  R_cam_to_body << 0, -1, 0,
                   1,  0, 0,
                   0,  0, 1;
  Eigen::Quaterniond quat_cam_to_body(R_cam_to_body);

  Eigen::Vector3d vehicle_pos = snapshot.local_position_ned.cast<double>();
  Eigen::Affine3d drone_transform;

  // Use full 3D attitude quaternion when available (compensating for vehicle pitch and roll tilts)
  if (snapshot.attitude_valid) {
    Eigen::Quaterniond vehicle_orientation = snapshot.attitude.cast<double>();
    drone_transform = Eigen::Translation3d(vehicle_pos) * vehicle_orientation;
  } else {
    double yaw = static_cast<double>(snapshot.heading);
    Eigen::Quaterniond vehicle_orientation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    drone_transform = Eigen::Translation3d(vehicle_pos) * vehicle_orientation;
  }

  Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_cam_to_body;

  Eigen::Vector3d tag_pos(tag_pose_optical.position.x, tag_pose_optical.position.y, tag_pose_optical.position.z);
  Eigen::Quaterniond tag_quat(
    tag_pose_optical.orientation.w,
    tag_pose_optical.orientation.x,
    tag_pose_optical.orientation.y,
    tag_pose_optical.orientation.z);
  Eigen::Affine3d tag_transform = Eigen::Translation3d(tag_pos) * tag_quat;

  Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

  WorldTargetTag result;
  result.position = tag_world_transform.translation();
  result.orientation = Eigen::Quaterniond(tag_world_transform.rotation());
  result.timestamp_ns = snapshot.monotonic_timestamp_ns;
  result.valid = true;
  return result;
}

Eigen::Vector2f PrecisionLandStrategy::calculate_velocity_setpoint_xy(
  const Eigen::Vector3d & vehicle_pos_ned,
  const Eigen::Vector3d & tag_pos_ned,
  float dt_s)
{
  float p_gain = vel_p_gain_;
  float i_gain = vel_i_gain_;

  // Position error
  float delta_pos_x = static_cast<float>(vehicle_pos_ned.x() - tag_pos_ned.x());
  float delta_pos_y = static_cast<float>(vehicle_pos_ned.y() - tag_pos_ned.y());

  // Discrete Integrator with dt_s scaling and anti-windup clamping
  vel_x_integral_ += delta_pos_x * dt_s;
  vel_y_integral_ += delta_pos_y * dt_s;
  float max_integral = max_velocity_;
  vel_x_integral_ = std::clamp(vel_x_integral_, -max_integral, max_integral);
  vel_y_integral_ = std::clamp(vel_y_integral_, -max_integral, max_integral);

  float Xp = delta_pos_x * p_gain;
  float Xi = vel_x_integral_ * i_gain;
  float Yp = delta_pos_y * p_gain;
  float Yi = vel_y_integral_ * i_gain;

  // Velocity setpoint computation
  float vx = -1.0f * (Xp + Xi);
  float vy = -1.0f * (Yp + Yi);

  // Clamping to strict prototype max_velocity (3.0 m/s)
  vx = std::clamp(vx, -max_velocity_, max_velocity_);
  vy = std::clamp(vy, -max_velocity_, max_velocity_);

  return Eigen::Vector2f(vx, vy);
}

std::vector<Eigen::Vector3f> PrecisionLandStrategy::generate_search_waypoints(
  const Eigen::Vector3f & start_pos_ned)
{
  double start_x = start_pos_ned.x();
  double start_y = start_pos_ned.y();
  double current_z = start_pos_ned.z();
  double min_z = -1.0;

  double max_radius = 2.0;
  double layer_spacing = 0.5;
  int points_per_layer = 16;
  std::vector<Eigen::Vector3f> waypoints;

  int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1
    ? 1
    : (static_cast<int>((min_z - current_z) / layer_spacing) / 2);

  for (int layer = 0; layer < num_layers; ++layer) {
    std::vector<Eigen::Vector3f> layer_waypoints;
    double radius = 0.0;

    for (int point = 0; point < points_per_layer + 1; ++point) {
      double angle = 2.0 * M_PI * point / points_per_layer;
      double x = start_x + radius * std::cos(angle);
      double y = start_y + radius * std::sin(angle);
      double z = current_z;

      layer_waypoints.push_back(Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
      radius += max_radius / points_per_layer;
    }

    waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());
    current_z += layer_spacing;

    std::reverse(layer_waypoints.begin(), layer_waypoints.end());
    for (auto & wp : layer_waypoints) {
      wp.z() = static_cast<float>(current_z);
    }
    waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());
    current_z += layer_spacing;
  }

  search_waypoints_ = waypoints;
  return search_waypoints_;
}

}  // namespace full_self_driving::flight
