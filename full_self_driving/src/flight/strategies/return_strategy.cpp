#include "flight/strategies/return_strategy.hpp"
#include <cmath>

namespace full_self_driving::flight
{

ReturnStrategy::ReturnStrategy(
  rclcpp::Node & node,
  px4_ros2::Context & context,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  std::shared_ptr<persistence::PersistenceManager> persistence,
  std::shared_ptr<domain::MissionContext> mission_ctx,
  ReturnMode return_mode,
  double return_altitude_above_home_m)
: node_(node),
  goto_setpoint_(std::make_shared<px4_ros2::GotoGlobalSetpointType>(context)),
  traj_setpoint_(std::make_shared<px4_ros2::TrajectorySetpointType>(context)),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  mission_ctx_(std::move(mission_ctx)),
  return_mode_(return_mode),
  return_altitude_above_home_m_(return_altitude_above_home_m)
{
}

ReturnStrategy::ReturnStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<px4_ros2::TrajectorySetpointType> traj_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  std::shared_ptr<persistence::PersistenceManager> persistence,
  std::shared_ptr<domain::MissionContext> mission_ctx,
  ReturnMode return_mode,
  double return_altitude_above_home_m)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  traj_setpoint_(std::move(traj_setpoint)),
  state_cache_(std::move(state_cache)),
  persistence_(std::move(persistence)),
  mission_ctx_(std::move(mission_ctx)),
  return_mode_(return_mode),
  return_altitude_above_home_m_(return_altitude_above_home_m)
{
}

void ReturnStrategy::on_enter()
{
  RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Entered ReturnStrategy. Mode: %d", static_cast<int>(return_mode_));
  completed_ = false;
  failed_ = false;
  failure_reason_.clear();
  dwell_timer_s_ = 0.0f;
  hover_settle_timer_s_ = 0.0f;
  home_initialized_ = false;
  heading_locked_ = false;
  hold_heading_rad_ = 0.0f;

  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
      hold_heading_rad_ = snapshot.heading;
      heading_locked_ = true;
    }
  }

  if (return_mode_ == ReturnMode::LAND_IMMEDIATELY) {
    sub_phase_ = SubPhase::DESCEND_HOME;
  } else if (return_mode_ == ReturnMode::HOLD_AT_FINAL_WAYPOINT) {
    sub_phase_ = SubPhase::APPROACH_HOME;
  } else {
    sub_phase_ = SubPhase::APPROACH_HOME;
  }
}

void ReturnStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Exited ReturnStrategy");
}

void ReturnStrategy::on_update(float dt_s)
{
  if (failed_) {
    return;
  }

  // Keep streaming zero-velocity setpoint with locked heading after touchdown
  // until the vehicle is fully disarmed. This prevents setpoint starvation and avoids
  // PX4 Offboard Loss / Safe Recovery Failsafe from triggering on the ground.
  if (completed_) {
    if (goto_setpoint_ && home_initialized_) {
      Eigen::Vector3d target{home_lat_, home_lon_, home_alt_msl_};
      goto_setpoint_->update(target, hold_heading_rad_, 0.0f, 0.0f, 0.0f);
    }
    if (traj_setpoint_) {
      traj_setpoint_->update(
        Eigen::Vector3f{0.0f, 0.0f, 0.0f},
        std::nullopt,
        hold_heading_rad_);
    }
    return;
  }

  if (!state_cache_) {
    fail("PX4 state cache is missing");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();

  if (!heading_locked_ && snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
    hold_heading_rad_ = snapshot.heading;
    heading_locked_ = true;
  }

  if (!home_initialized_) {
    if (mission_ctx_ && mission_ctx_->has_origin_home_position()) {
      auto origin = mission_ctx_->get_origin_home_position();
      home_lat_ = origin.latitude_deg;
      home_lon_ = origin.longitude_deg;
      home_alt_msl_ = origin.altitude_msl_m;
      home_initialized_ = true;
      RCLCPP_INFO(
        node_.get_logger(),
        "[RETURN_STRATEGY] Using locked Sortie Origin Home Base: lat=%.6f, lon=%.6f, alt=%.2f m",
        home_lat_, home_lon_, home_alt_msl_);
    } else if (snapshot.home_pos_valid) {
      home_lat_ = snapshot.home_global_position.x();
      home_lon_ = snapshot.home_global_position.y();
      home_alt_msl_ = snapshot.home_global_position.z();
      home_initialized_ = true;
    } else if (snapshot.global_pos_valid) {
      home_lat_ = snapshot.global_position.x();
      home_lon_ = snapshot.global_position.y();
      home_alt_msl_ = snapshot.global_position.z();
      home_initialized_ = true;
    }
  }

  if (return_mode_ == ReturnMode::HOLD_AT_FINAL_WAYPOINT) {
    if (goto_setpoint_ && snapshot.global_pos_valid) {
      Eigen::Vector3d hold_target{snapshot.global_position.x(), snapshot.global_position.y(), snapshot.global_position.z()};
      goto_setpoint_->update(hold_target, hold_heading_rad_, 0.0f, 0.0f, 0.0f);
    }
    return;
  }

  if (sub_phase_ == SubPhase::APPROACH_HOME) {
    if (home_initialized_ && goto_setpoint_) {
      double target_alt = home_alt_msl_ + return_altitude_above_home_m_;
      Eigen::Vector3d target{home_lat_, home_lon_, target_alt};

      float h_dist = px4_ros2::horizontalDistanceToGlobalPosition(snapshot.global_position, target);
      float h_speed = std::sqrt(snapshot.local_velocity_ned.x() * snapshot.local_velocity_ned.x() +
                                snapshot.local_velocity_ned.y() * snapshot.local_velocity_ned.y());

      // Update course heading while in transit at sufficient horizontal speed
      if (h_speed >= 0.3f && snapshot.local_pos_valid) {
        hold_heading_rad_ = std::atan2(snapshot.local_velocity_ned.y(), snapshot.local_velocity_ned.x());
        heading_locked_ = true;
      }

      // Progressive velocity braking to eliminate forward overshoot:
      float approach_speed = 3.0f;
      if (std::isfinite(h_dist) && h_dist < 6.0f) {
        approach_speed = std::clamp(h_dist * 0.5f, 0.4f, 2.0f);
      }
      goto_setpoint_->update(target, hold_heading_rad_, approach_speed, 1.0f, 0.785f);

      // Strict zero-momentum hover gate (< 35cm error and < 0.20 m/s ground speed):
      if (std::isfinite(h_dist) && h_dist <= 0.35f && h_speed <= 0.20f) {
        hover_settle_timer_s_ += dt_s;
        if (hover_settle_timer_s_ >= kHoverSettleDurationS) {
          // Lock heading permanently for the entire descent and touchdown sequence
          if (snapshot.local_pos_valid && std::isfinite(snapshot.heading)) {
            hold_heading_rad_ = snapshot.heading;
          }
          heading_locked_ = true;
          RCLCPP_INFO(
            node_.get_logger(),
            "[RETURN_STRATEGY] Zero-momentum hover STABILIZED over Home Base (dist=%.2f m, speed=%.2f m/s, heading=%.2f deg). Starting 2-stage descent...",
            h_dist, h_speed, hold_heading_rad_ * 180.0f / static_cast<float>(M_PI));
          sub_phase_ = SubPhase::DESCEND_HOME;
        }
      } else {
        hover_settle_timer_s_ = 0.0f;
      }
    } else {
      sub_phase_ = SubPhase::DESCEND_HOME;
    }
  }

  if (sub_phase_ == SubPhase::DESCEND_HOME) {
    double current_alt_agl = 0.0;
    if (home_initialized_ && snapshot.global_pos_valid) {
      current_alt_agl = snapshot.global_position.z() - home_alt_msl_;
    } else {
      current_alt_agl = -static_cast<double>(snapshot.local_position_ned.z());
    }

    double approach_threshold_m = 2.5;
    if (mission_ctx_ && mission_ctx_->get_resolved_config()) {
      approach_threshold_m = mission_ctx_->get_resolved_config()->routes.approach_altitude_m;
    }

    // 2-Stage Descent: 1.0 m/s above 2.5m, 0.35 m/s soft touchdown below 2.5m
    float descent_speed = (current_alt_agl > approach_threshold_m) ? 1.0f : 0.35f;

    // Closed-loop horizontal position guidance locked to exact (home_lat_, home_lon_) with locked heading:
    if (goto_setpoint_ && home_initialized_) {
      Eigen::Vector3d target{home_lat_, home_lon_, home_alt_msl_};
      goto_setpoint_->update(target, hold_heading_rad_, 0.5f, descent_speed);
    } else if (traj_setpoint_) {
      traj_setpoint_->update(
        Eigen::Vector3f{0.0f, 0.0f, descent_speed},
        std::nullopt,
        hold_heading_rad_);
    }

    float vz = snapshot.local_velocity_ned.z();
    if (snapshot.is_landed || (snapshot.local_position_ned.z() >= -0.1f && std::abs(vz) < 0.15f)) {
      RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Touchdown at Home Base detected! Dwell verification starting...");
      sub_phase_ = SubPhase::TOUCHDOWN_DWELL;
      dwell_timer_s_ = 0.0f;
    }
  }

  if (sub_phase_ == SubPhase::TOUCHDOWN_DWELL) {
    if (goto_setpoint_ && home_initialized_) {
      Eigen::Vector3d target{home_lat_, home_lon_, home_alt_msl_};
      goto_setpoint_->update(target, hold_heading_rad_, 0.0f, 0.0f, 0.0f);
    }
    if (traj_setpoint_) {
      traj_setpoint_->update(
        Eigen::Vector3f{0.0f, 0.0f, 0.0f},
        std::nullopt,
        hold_heading_rad_);
    }
    dwell_timer_s_ += dt_s;
    if (dwell_timer_s_ >= kTouchdownDwellDurationS) {
      RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Touchdown dwell complete! Sortie finished successfully.");

      // Checkpoint completion
      if (persistence_) {
        persistence::JournalEntry entry;
        entry.event_id = "EVT_SORTIE_COMPLETED";
        entry.mission_id = mission_ctx_ ? mission_ctx_->get_mission_id() : "";
        entry.sortie_id = mission_ctx_ ? mission_ctx_->get_sortie_id() : "";
        entry.component = "return_strategy";
        entry.detail = "Sortie completed successfully with safe touchdown at home airfield";
        entry.timestamp_monotonic_ns = snapshot.monotonic_timestamp_ns;
        persistence_->append_journal_entry(entry);
      }

      completed_ = true;
      sub_phase_ = SubPhase::FINISHED;
    }
  }
}

void ReturnStrategy::fail(const std::string & reason)
{
  if (completed_ || failed_) {
    return;
  }

  failed_ = true;
  failure_reason_ = reason;
  RCLCPP_ERROR(node_.get_logger(), "[RETURN_STRATEGY] ReturnStrategy failed: %s", reason.c_str());
}

}  // namespace full_self_driving::flight
