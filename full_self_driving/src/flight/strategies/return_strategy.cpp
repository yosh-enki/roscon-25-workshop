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
  home_initialized_ = false;

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
  if (completed_ || failed_) {
    return;
  }

  if (!state_cache_) {
    fail("PX4 state cache is missing");
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();

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
      goto_setpoint_->update(hold_target, std::nullopt, 0.0f, 0.0f, 0.0f);
    }
    return;
  }

  if (sub_phase_ == SubPhase::APPROACH_HOME) {
    if (home_initialized_ && goto_setpoint_) {
      double target_alt = home_alt_msl_ + return_altitude_above_home_m_;
      Eigen::Vector3d target{home_lat_, home_lon_, target_alt};
      goto_setpoint_->update(target, std::nullopt, 3.0f, 1.0f, 0.785f);

      float h_dist = px4_ros2::horizontalDistanceToGlobalPosition(snapshot.global_position, target);
      float h_speed = std::sqrt(snapshot.local_velocity_ned.x() * snapshot.local_velocity_ned.x() +
                                snapshot.local_velocity_ned.y() * snapshot.local_velocity_ned.y());
      if (std::isfinite(h_dist) && h_dist <= 2.0f && h_speed <= 0.75f) {
        RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Arrived and settled over Sortie Home Base (dist=%.2f m, speed=%.2f m/s). Starting descent...",
          h_dist, h_speed);
        sub_phase_ = SubPhase::DESCEND_HOME;
      }
    } else {
      sub_phase_ = SubPhase::DESCEND_HOME;
    }
  }

  if (sub_phase_ == SubPhase::DESCEND_HOME) {
    if (traj_setpoint_) {
      // 0.35 m/s soft vertical descent
      traj_setpoint_->update(
        Eigen::Vector3f{0.0f, 0.0f, 0.35f},
        std::nullopt,
        std::nullopt);
    } else if (goto_setpoint_ && home_initialized_) {
      Eigen::Vector3d target{home_lat_, home_lon_, home_alt_msl_};
      goto_setpoint_->update(target, std::nullopt, 0.35f);
    }

    float vz = snapshot.local_velocity_ned.z();
    if (snapshot.is_landed || (snapshot.local_position_ned.z() >= -0.3f && std::abs(vz) < 0.25f)) {
      RCLCPP_INFO(node_.get_logger(), "[RETURN_STRATEGY] Touchdown at Home Base detected! Dwell verification starting...");
      sub_phase_ = SubPhase::TOUCHDOWN_DWELL;
      dwell_timer_s_ = 0.0f;
    }
  }

  if (sub_phase_ == SubPhase::TOUCHDOWN_DWELL) {
    if (traj_setpoint_) {
      traj_setpoint_->update(
        Eigen::Vector3f{0.0f, 0.0f, 0.0f},
        std::nullopt,
        std::nullopt);
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
