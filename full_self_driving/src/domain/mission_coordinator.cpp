#include "domain/mission_coordinator.hpp"
#include "flight/strategies/takeoff_strategy.hpp"
#include "flight/strategies/transit_in_strategy.hpp"

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

}  // namespace full_self_driving::domain
