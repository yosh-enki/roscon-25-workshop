#include "flight/strategies/search_strategy.hpp"
#include <cmath>

namespace full_self_driving::flight
{

SearchStrategy::SearchStrategy(
  rclcpp::Node & node,
  px4_ros2::Context & context,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  std::shared_ptr<runtime::PlanManager> plan_manager,
  domain::WorkingPlan working_plan,
  double search_altitude_m,
  float max_horizontal_speed_m_s,
  float waypoint_reach_radius_m,
  float max_yaw_rate_rad_s,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  goto_setpoint_(std::make_shared<px4_ros2::GotoGlobalSetpointType>(context)),
  state_cache_(std::move(state_cache)),
  plan_manager_(std::move(plan_manager)),
  persistence_(std::move(persistence)),
  working_plan_(std::move(working_plan)),
  search_altitude_m_(search_altitude_m),
  waypoint_reach_radius_m_(waypoint_reach_radius_m),
  max_horizontal_speed_m_s_(max_horizontal_speed_m_s),
  max_yaw_rate_rad_s_(max_yaw_rate_rad_s)
{
  if (!std::isfinite(search_altitude_m_) || search_altitude_m_ <= 0.0 ||
      !std::isfinite(max_horizontal_speed_m_s_) || max_horizontal_speed_m_s_ <= 0.0f ||
      !std::isfinite(waypoint_reach_radius_m_) || waypoint_reach_radius_m_ <= 0.0f ||
      !std::isfinite(max_yaw_rate_rad_s_) || max_yaw_rate_rad_s_ <= 0.0f) {
    parameters_valid_ = false;
    fail("SearchStrategy parameters must be finite and positive");
  }
}

SearchStrategy::SearchStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  std::shared_ptr<runtime::PlanManager> plan_manager,
  domain::WorkingPlan working_plan,
  double search_altitude_m,
  float max_horizontal_speed_m_s,
  float waypoint_reach_radius_m,
  float max_yaw_rate_rad_s,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  state_cache_(std::move(state_cache)),
  plan_manager_(std::move(plan_manager)),
  persistence_(std::move(persistence)),
  working_plan_(std::move(working_plan)),
  search_altitude_m_(search_altitude_m),
  waypoint_reach_radius_m_(waypoint_reach_radius_m),
  max_horizontal_speed_m_s_(max_horizontal_speed_m_s),
  max_yaw_rate_rad_s_(max_yaw_rate_rad_s)
{
  if (!std::isfinite(search_altitude_m_) || search_altitude_m_ <= 0.0 ||
      !std::isfinite(max_horizontal_speed_m_s_) || max_horizontal_speed_m_s_ <= 0.0f ||
      !std::isfinite(waypoint_reach_radius_m_) || waypoint_reach_radius_m_ <= 0.0f ||
      !std::isfinite(max_yaw_rate_rad_s_) || max_yaw_rate_rad_s_ <= 0.0f) {
    parameters_valid_ = false;
    fail("SearchStrategy parameters must be finite and positive");
  }
}

SearchStrategy::SearchStrategy(
  rclcpp::Node & node,
  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> goto_setpoint,
  std::shared_ptr<adapters::Px4StateCache> state_cache,
  domain::CanonicalSearchRoute route,
  double search_altitude_m,
  float max_horizontal_speed_m_s,
  float waypoint_reach_radius_m,
  float max_yaw_rate_rad_s)
: node_(node),
  goto_setpoint_(std::move(goto_setpoint)),
  state_cache_(std::move(state_cache)),
  plan_manager_(nullptr),
  persistence_(nullptr),
  route_(std::move(route)),
  search_altitude_m_(search_altitude_m),
  waypoint_reach_radius_m_(waypoint_reach_radius_m),
  max_horizontal_speed_m_s_(max_horizontal_speed_m_s),
  max_yaw_rate_rad_s_(max_yaw_rate_rad_s)
{
  if (!std::isfinite(search_altitude_m_) || search_altitude_m_ <= 0.0 ||
      !std::isfinite(max_horizontal_speed_m_s_) || max_horizontal_speed_m_s_ <= 0.0f ||
      !std::isfinite(waypoint_reach_radius_m_) || waypoint_reach_radius_m_ <= 0.0f ||
      !std::isfinite(max_yaw_rate_rad_s_) || max_yaw_rate_rad_s_ <= 0.0f) {
    parameters_valid_ = false;
    fail("SearchStrategy parameters must be finite and positive");
  }
  total_source_waypoints_ = static_cast<uint32_t>(route_.waypoints.size());
}

void SearchStrategy::fail(const std::string & reason)
{
  failed_ = true;
  failure_reason_ = reason;
  RCLCPP_ERROR(node_.get_logger(), "[SEARCH] Strategy failure: %s", reason.c_str());
  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = "SEARCH_STRATEGY_FAILED";
    entry.detail = reason;
    persistence_->append_journal_entry(entry);
  }
}

bool SearchStrategy::data_timed_out() const
{
  if (activation_time_.time_since_epoch().count() == 0) {
    return false;
  }
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
    std::chrono::steady_clock::now() - activation_time_).count();
  if (elapsed < 1.0f) {
    return false;
  }
  if (!state_cache_) {
    return true;
  }
  auto snapshot = state_cache_->capture_snapshot();
  return !snapshot.global_pos_valid || !snapshot.local_pos_valid;
}

void SearchStrategy::on_enter()
{
  current_waypoint_index_ = 0;
  target_altitude_amsl_m_ = 0.0;
  target_altitude_set_ = false;
  mode_finished_ = false;
  failed_ = false;
  failure_reason_.clear();
  activation_time_ = std::chrono::steady_clock::now();

  if (!parameters_valid_) {
    fail("SearchStrategy enter aborted due to invalid configuration parameters");
    return;
  }

  // If working plan is present, refresh it and derive the search route
  if (plan_manager_ && !working_plan_.get_working_plan_id().empty()) {
    auto refreshed = plan_manager_->get_working_plan(working_plan_.get_working_plan_id());
    if (refreshed) {
      working_plan_ = *refreshed;
    }
  }

  if (!working_plan_.get_working_plan_id().empty()) {
    route_ = working_plan_.route_for_search();
    total_source_waypoints_ = static_cast<uint32_t>(working_plan_.get_source_route().waypoints.size());
    if (total_source_waypoints_ == 0 && !route_.waypoints.empty()) {
      total_source_waypoints_ = static_cast<uint32_t>(route_.waypoints.size());
    }
    starts_with_entry_point_ = working_plan_.get_checkpoint().has_checkpoint_position;
    first_plan_waypoint_source_index_ = working_plan_.get_checkpoint().next_source_index;
  } else if (route_.waypoints.empty()) {
    fail("Search route is empty and no valid working plan was provided");
    return;
  } else {
    total_source_waypoints_ = static_cast<uint32_t>(route_.waypoints.size());
    starts_with_entry_point_ = false;
    first_plan_waypoint_source_index_ = 0;
  }

  if (route_.waypoints.empty()) {
    fail("Working plan generated an empty search route");
    return;
  }

  // Setup target altitude AMSL based on home position
  if (state_cache_) {
    auto snapshot = state_cache_->capture_snapshot();
    if (snapshot.home_pos_valid) {
      home_altitude_msl_m_ = snapshot.home_global_position.z();
      target_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
      target_altitude_set_ = true;
    } else if (snapshot.global_pos_valid) {
      home_altitude_msl_m_ = snapshot.global_position.z();
      target_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
      target_altitude_set_ = true;
    }
  }

  if (!target_altitude_set_) {
    target_altitude_amsl_m_ = search_altitude_m_;
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[SEARCH] Search strategy activated with %zu route waypoint(s) (total source wp: %u, starts_with_entry: %d, first_plan_idx: %u), target alt: %.2f m AMSL",
    route_.waypoints.size(), total_source_waypoints_, starts_with_entry_point_, first_plan_waypoint_source_index_, target_altitude_amsl_m_);

  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = "SEARCH_STRATEGY_ACTIVATED";
    entry.detail = "Activated with " + std::to_string(route_.waypoints.size()) + " search waypoints";
    persistence_->append_journal_entry(entry);
  }
}

void SearchStrategy::on_update(float dt_s)
{
  (void)dt_s;
  if (failed_) {
    return;
  }

  if (!state_cache_ || !goto_setpoint_) {
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();
  if (!snapshot.global_pos_valid || !snapshot.local_pos_valid) {
    if (data_timed_out()) {
      fail("PX4 global/local position data timeout during Search");
    }
    return;
  }

  if (!target_altitude_set_ && snapshot.home_pos_valid) {
    home_altitude_msl_m_ = snapshot.home_global_position.z();
    target_altitude_amsl_m_ = home_altitude_msl_m_ + search_altitude_m_;
    target_altitude_set_ = true;
  }

  if (mode_finished_) {
    if (!route_.waypoints.empty()) {
      const auto & last_wp = route_.waypoints.back();
      Eigen::Vector3d hold_target(last_wp.latitude_deg, last_wp.longitude_deg, target_altitude_amsl_m_);
      goto_setpoint_->update(hold_target);
    }
    return;
  }

  if (route_.waypoints.empty()) {
    fail("Search route has no waypoints to navigate");
    return;
  }

  // 1. Climb to configured search altitude from current XY position first (Prototype baseline)
  double current_alt_amsl = snapshot.global_position.z();
  if (current_alt_amsl < target_altitude_amsl_m_ - altitude_tolerance_m_) {
    Eigen::Vector3d climb_target(
      snapshot.global_position.x(),
      snapshot.global_position.y(),
      target_altitude_amsl_m_);
    float heading = snapshot.heading;
    goto_setpoint_->update(
      climb_target,
      heading,
      max_horizontal_speed_m_s_,
      max_vertical_speed_m_s_,
      max_yaw_rate_rad_s_);
    return;
  }

  // 2. Waypoint progression
  Eigen::Vector2d current_pos_2d(snapshot.global_position.x(), snapshot.global_position.y());
  if (current_waypoint_index_ < route_.waypoints.size()) {
    const auto & target_wp = route_.waypoints[current_waypoint_index_];
    Eigen::Vector2d target_pos_2d(target_wp.latitude_deg, target_wp.longitude_deg);
    float distance = px4_ros2::horizontalDistanceToGlobalPosition(current_pos_2d, target_pos_2d);

    if (distance < waypoint_reach_radius_m_) {
      RCLCPP_INFO(
        node_.get_logger(),
        "[SEARCH] Reached active search waypoint %zu/%zu",
        current_waypoint_index_ + 1, route_.waypoints.size());

      // Compute next_source_index according to prototype SearchPlanner formula
      uint32_t next_source_idx = first_plan_waypoint_source_index_;
      if (starts_with_entry_point_) {
        if (current_waypoint_index_ > 0) {
          next_source_idx += static_cast<uint32_t>(current_waypoint_index_);
        }
      } else {
        next_source_idx += static_cast<uint32_t>(current_waypoint_index_ + 1);
      }
      if (next_source_idx > total_source_waypoints_) {
        next_source_idx = total_source_waypoints_;
      }

      uint32_t completed_count = next_source_idx;
      float progress = 0.0f;
      if (total_source_waypoints_ > 0) {
        progress = (static_cast<float>(completed_count) / static_cast<float>(total_source_waypoints_)) * 100.0f;
      }

      domain::SearchCheckpointData cp;
      cp.working_plan_id = working_plan_.get_working_plan_id();
      cp.generation = working_plan_.get_generation();
      cp.next_source_index = next_source_idx;
      cp.completed_waypoints = completed_count;
      cp.total_waypoints = total_source_waypoints_;
      cp.progress_percent = progress;
      cp.has_checkpoint_position = false;
      cp.checkpoint_latitude_deg = target_wp.latitude_deg;
      cp.checkpoint_longitude_deg = target_wp.longitude_deg;
      cp.checkpoint_altitude_m = search_altitude_m_;
      cp.checkpoint_reason = "WAYPOINT_SETTLED";

      if (plan_manager_ && !working_plan_.get_working_plan_id().empty()) {
        plan_manager_->update_checkpoint(working_plan_.get_working_plan_id(), cp, "WAYPOINT_SETTLED");
      }
      working_plan_.update_checkpoint(cp, "WAYPOINT_SETTLED");

      if (persistence_) {
        persistence::JournalEntry entry;
        entry.event_id = "SEARCH_WAYPOINT_SETTLED";
        entry.detail = "Reached waypoint " + std::to_string(current_waypoint_index_ + 1) +
                       " (progress " + std::to_string(static_cast<int>(progress)) + "%)";
        persistence_->append_journal_entry(entry);
      }

      if (checkpoint_cb_) {
        checkpoint_cb_(cp);
      }
      if (waypoint_cb_) {
        waypoint_cb_(current_waypoint_index_, true);
      }

      ++current_waypoint_index_;
    }
  }

  // 3. Final waypoint hold and completion
  if (current_waypoint_index_ >= route_.waypoints.size()) {
    mode_finished_ = true;
    const auto & last_wp = route_.waypoints.back();
    Eigen::Vector3d hold_target(last_wp.latitude_deg, last_wp.longitude_deg, target_altitude_amsl_m_);
    goto_setpoint_->update(hold_target);

    RCLCPP_INFO(
      node_.get_logger(),
      "[SEARCH] All search waypoints completed. Holding over final waypoint (lat=%.7f, lon=%.7f)",
      last_wp.latitude_deg, last_wp.longitude_deg);

    if (completion_cb_) {
      completion_cb_(true);
    }
    return;
  }

  // 4. Update setpoint towards active target waypoint
  const auto & active_wp = route_.waypoints[current_waypoint_index_];
  Eigen::Vector3d target_pos(active_wp.latitude_deg, active_wp.longitude_deg, target_altitude_amsl_m_);
  Eigen::Vector2d active_pos_2d(active_wp.latitude_deg, active_wp.longitude_deg);
  float dist_to_wp = px4_ros2::horizontalDistanceToGlobalPosition(current_pos_2d, active_pos_2d);

  if (dist_to_wp < 0.1f) {
    goto_setpoint_->update(target_pos);
  } else {
    float heading = px4_ros2::headingToGlobalPosition(current_pos_2d, active_pos_2d);
    goto_setpoint_->update(
      target_pos,
      heading,
      max_horizontal_speed_m_s_,
      max_vertical_speed_m_s_,
      max_yaw_rate_rad_s_);
  }
}

void SearchStrategy::record_safe_deactivation_checkpoint()
{
  if (!state_cache_) {
    return;
  }

  auto snapshot = state_cache_->capture_snapshot();
  if (!snapshot.global_pos_valid) {
    RCLCPP_WARN(
      node_.get_logger(),
      "[SEARCH] Search deactivated without valid global position; working plan deactivation checkpoint position unavailable");
    if (persistence_) {
      persistence::JournalEntry entry;
      entry.event_id = "CHECKPOINT_NOT_SAVED_POSITION_UNAVAILABLE";
      entry.detail = "Search deactivated but vehicle global position was invalid";
      persistence_->append_journal_entry(entry);
    }
    return;
  }

  uint32_t next_source_idx = first_plan_waypoint_source_index_;
  if (starts_with_entry_point_) {
    if (current_waypoint_index_ > 0) {
      next_source_idx += static_cast<uint32_t>(current_waypoint_index_ - 1);
    }
  } else {
    next_source_idx += static_cast<uint32_t>(current_waypoint_index_);
  }
  if (next_source_idx > total_source_waypoints_) {
    next_source_idx = total_source_waypoints_;
  }

  uint32_t completed_count = next_source_idx;
  float progress = 0.0f;
  if (total_source_waypoints_ > 0) {
    progress = (static_cast<float>(completed_count) / static_cast<float>(total_source_waypoints_)) * 100.0f;
  }

  domain::SearchCheckpointData cp;
  cp.working_plan_id = working_plan_.get_working_plan_id();
  cp.generation = working_plan_.get_generation();
  cp.next_source_index = next_source_idx;
  cp.completed_waypoints = completed_count;
  cp.total_waypoints = total_source_waypoints_;
  cp.progress_percent = progress;
  cp.has_checkpoint_position = true;
  cp.checkpoint_latitude_deg = snapshot.global_position.x();
  cp.checkpoint_longitude_deg = snapshot.global_position.y();
  cp.checkpoint_altitude_m = search_altitude_m_;
  cp.checkpoint_reason = "SAFE_DEACTIVATION";

  if (plan_manager_ && !working_plan_.get_working_plan_id().empty()) {
    plan_manager_->update_checkpoint(working_plan_.get_working_plan_id(), cp, "SAFE_DEACTIVATION");
  }
  working_plan_.update_checkpoint(cp, "SAFE_DEACTIVATION");

  if (persistence_) {
    persistence::JournalEntry entry;
    entry.event_id = "SEARCH_CHECKPOINT_SAVED";
    entry.detail = "Saved deactivation checkpoint at lat=" + std::to_string(cp.checkpoint_latitude_deg) +
                   " lon=" + std::to_string(cp.checkpoint_longitude_deg) +
                   " next_idx=" + std::to_string(next_source_idx);
    persistence_->append_journal_entry(entry);
  }

  if (checkpoint_cb_) {
    checkpoint_cb_(cp);
  }

  RCLCPP_INFO(
    node_.get_logger(),
    "[SEARCH] Safe deactivation checkpoint saved: lat=%.7f, lon=%.7f, next_source_idx=%u, progress=%.1f%%",
    cp.checkpoint_latitude_deg, cp.checkpoint_longitude_deg, next_source_idx, progress);
}

void SearchStrategy::on_exit()
{
  record_safe_deactivation_checkpoint();
  RCLCPP_INFO(node_.get_logger(), "[SEARCH] Search strategy deactivated");
}

}  // namespace full_self_driving::flight
