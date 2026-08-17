#include "domain/mission_coordinator.hpp"
#include "flight/strategies/takeoff_strategy.hpp"
#include "flight/strategies/transit_in_strategy.hpp"
#include "flight/strategies/search_strategy.hpp"
#include <cmath>

namespace full_self_driving::domain
{

MissionCoordinator::MissionCoordinator(std::shared_ptr<MissionContext> context)
: context_(std::move(context))
{
  transition_trace_.push_back("INIT -> WAITING_FOR_MODE");
}

void MissionCoordinator::bind_executor(
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor,
  std::shared_ptr<flight::FullSelfDrivingMode> mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  executor_ = std::move(executor);
  mode_ = std::move(mode);
}

void MissionCoordinator::set_plan_manager(std::shared_ptr<runtime::PlanManager> pm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_manager_ = std::move(pm);
}

std::shared_ptr<runtime::PlanManager> MissionCoordinator::get_plan_manager() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_manager_;
}

void MissionCoordinator::handle_target_lock_update(const LiveTargetLock & lock)
{
  std::lock_guard<std::mutex> guard(mutex_);
  // Perception only publishes observations and lock decisions (Property 12).
  // The coordinator evaluates domain rules before deciding if any transition is needed.
  if (lock.is_qualified()) {
    transition_trace_.push_back("LOCK_QUALIFIED: id=" + std::to_string(lock.identity.marker_id));
  } else if (lock.is_lost()) {
    transition_trace_.push_back("LOCK_LOST");
  }
}

void MissionCoordinator::handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason reason)
{
  std::lock_guard<std::mutex> guard(mutex_);
  takeover_active_ = true;
  std::string reason_str = (reason == flight::FullSelfDrivingModeExecutor::DeactivateReason::FailsafeActivated)
    ? "FailsafeActivated" : "ManualTakeover";
  transition_trace_.push_back("TAKEOVER: " + reason_str);
  current_strategy_ = flight::StrategyType::HOLD;
}

void MissionCoordinator::handle_emergency_stop()
{
  std::lock_guard<std::mutex> guard(mutex_);
  emergency_stop_active_ = true;
  transition_trace_.push_back("EMERGENCY_STOP");
  current_strategy_ = flight::StrategyType::FAILSAFE;
}

bool MissionCoordinator::request_transition(flight::StrategyType next_strategy, std::string * out_error)
{
  std::lock_guard<std::mutex> guard(mutex_);

  if (emergency_stop_active_) {
    if (out_error) *out_error = "Emergency stop is active; transitions forbidden";
    return false;
  }

  if (takeover_active_ && next_strategy != flight::StrategyType::HOLD && next_strategy != flight::StrategyType::FAILSAFE) {
    if (out_error) *out_error = "Takeover active; manual control has authority";
    return false;
  }

  std::string trace_entry = flight::strategy_type_to_string(current_strategy_) +
    " -> " + flight::strategy_type_to_string(next_strategy);
  transition_trace_.push_back(trace_entry);

  if (next_strategy == flight::StrategyType::ACQUIRE_TARGET) {
    transition_trace_.push_back("ACQUIRE_TARGET -> SEARCH (Direct unavailable/fallback)");
  }

  current_strategy_ = next_strategy;

  if (mode_) {
    if (next_strategy == flight::StrategyType::WAITING_FOR_MODE) {
      mode_->set_strategy(std::make_unique<flight::WaitingForModeStrategy>());
    } else if (next_strategy == flight::StrategyType::TAKEOFF) {
      double takeoff_alt = 10.0;
      if (context_ && context_->get_resolved_config()) {
        takeoff_alt = context_->get_resolved_config()->routes.search_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::TakeoffStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), takeoff_alt));
    } else if (next_strategy == flight::StrategyType::TRANSIT_IN) {
      Route route;
      if (has_custom_transit_in_route_) {
        route = custom_transit_in_route_;
      } else {
        route = Route::create_default_kmitl_transit_in_route();
        if (context_ && context_->get_resolved_config()) {
          const auto & cfg = context_->get_resolved_config()->routes;
          route.set_max_horizontal_speed_m_s(static_cast<float>(cfg.transit_in_speed_m_s));
          route.set_transit_altitude_above_home_m(cfg.search_altitude_m);
          route.set_acceptance_radius_m(static_cast<float>(cfg.acceptance_radius_m));
          route.set_max_yaw_rate_deg_s(static_cast<float>(cfg.max_yaw_rate_deg_s));
        }
      }
      mode_->set_strategy(std::make_unique<flight::TransitInStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route));
    } else if (next_strategy == flight::StrategyType::SEARCH || next_strategy == flight::StrategyType::ACQUIRE_TARGET) {
      double search_alt = 15.0;
      float max_h_speed = 5.0f;
      float reach_rad = 4.0f;
      float max_yaw_rate = 0.785398163f;

      if (context_ && context_->get_resolved_config()) {
        const auto & cfg = context_->get_resolved_config()->routes;
        search_alt = cfg.search_altitude_m;
        max_h_speed = static_cast<float>(cfg.transit_in_speed_m_s);
        reach_rad = static_cast<float>(cfg.acceptance_radius_m);
        max_yaw_rate = static_cast<float>(cfg.max_yaw_rate_deg_s * M_PI / 180.0);
      }

      if (has_custom_search_plan_) {
        mode_->set_strategy(std::make_unique<flight::SearchStrategy>(
          mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
          plan_manager_, custom_search_plan_, search_alt, max_h_speed, reach_rad, max_yaw_rate));
      } else if (has_custom_search_route_) {
        mode_->set_strategy(std::make_unique<flight::SearchStrategy>(
          mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
          custom_search_route_, search_alt, max_h_speed, reach_rad, max_yaw_rate));
      } else if (plan_manager_) {
        std::string map_id = "kmitl_airfield";
        std::string scenario_id = "default_scenario";
        std::string wp_id = "";
        if (context_) {
          const auto & sel = context_->get_selection();
          map_id = sel.map_id;
          scenario_id = sel.scenario_id;
          wp_id = sel.working_plan_id;
        }
        std::optional<WorkingPlan> wp;
        if (!wp_id.empty()) {
          wp = plan_manager_->get_working_plan(wp_id);
        }
        if (!wp) {
          wp = plan_manager_->get_active_working_plan(map_id, scenario_id);
        }
        if (wp) {
          mode_->set_strategy(std::make_unique<flight::SearchStrategy>(
            mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
            plan_manager_, *wp, search_alt, max_h_speed, reach_rad, max_yaw_rate));
        } else {
          CanonicalSearchRoute default_search_route;
          default_search_route.default_altitude_m = static_cast<float>(search_alt);
          default_search_route.cruise_speed_m_s = max_h_speed;
          SearchWaypoint wp1{13.73132845, 100.78990948, search_alt, 0};
          SearchWaypoint wp2{13.73078947, 100.78783793, search_alt, 1};
          default_search_route.waypoints.push_back(wp1);
          default_search_route.waypoints.push_back(wp2);

          mode_->set_strategy(std::make_unique<flight::SearchStrategy>(
            mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
            default_search_route, search_alt, max_h_speed, reach_rad, max_yaw_rate));
        }
      } else {
        CanonicalSearchRoute default_search_route;
        default_search_route.default_altitude_m = static_cast<float>(search_alt);
        default_search_route.cruise_speed_m_s = max_h_speed;
        SearchWaypoint wp1{13.73132845, 100.78990948, search_alt, 0};
        SearchWaypoint wp2{13.73078947, 100.78783793, search_alt, 1};
        default_search_route.waypoints.push_back(wp1);
        default_search_route.waypoints.push_back(wp2);

        mode_->set_strategy(std::make_unique<flight::SearchStrategy>(
          mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
          default_search_route, search_alt, max_h_speed, reach_rad, max_yaw_rate));
      }
    }
  }

  return true;
}

uint8_t MissionCoordinator::get_flight_phase() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return static_cast<uint8_t>(current_strategy_);
}

void MissionCoordinator::clear_transition_trace()
{
  std::lock_guard<std::mutex> guard(mutex_);
  transition_trace_.clear();
}

void MissionCoordinator::set_custom_transit_in_route(const Route & route)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_transit_in_route_ = route;
  has_custom_transit_in_route_ = true;
}

void MissionCoordinator::reset_custom_transit_in_route()
{
  std::lock_guard<std::mutex> guard(mutex_);
  has_custom_transit_in_route_ = false;
}

void MissionCoordinator::set_custom_search_route(const CanonicalSearchRoute & route)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_search_route_ = route;
  has_custom_search_route_ = true;
  has_custom_search_plan_ = false;
}

void MissionCoordinator::set_custom_search_plan(const WorkingPlan & wp)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_search_plan_ = wp;
  has_custom_search_plan_ = true;
  has_custom_search_route_ = false;
}

void MissionCoordinator::reset_custom_search()
{
  std::lock_guard<std::mutex> guard(mutex_);
  has_custom_search_plan_ = false;
  has_custom_search_route_ = false;
}

}  // namespace full_self_driving::domain
