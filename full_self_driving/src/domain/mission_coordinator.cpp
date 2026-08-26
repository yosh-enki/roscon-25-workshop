#include "domain/mission_coordinator.hpp"
#include "flight/strategies/takeoff_strategy.hpp"
#include "flight/strategies/transit_in_strategy.hpp"
#include "flight/strategies/search_strategy.hpp"
#include "flight/strategies/direct_strategy.hpp"
#include "flight/strategies/precision_land_strategy.hpp"
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

void MissionCoordinator::set_pad_registry(std::shared_ptr<registry::PadRegistry> registry)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pad_registry_ = std::move(registry);
}

std::shared_ptr<registry::PadRegistry> MissionCoordinator::get_pad_registry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pad_registry_;
}

void MissionCoordinator::set_payload_controller(std::shared_ptr<payload::PayloadController> pc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  payload_controller_ = std::move(pc);
}

std::shared_ptr<payload::PayloadController> MissionCoordinator::get_payload_controller() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return payload_controller_;
}

void MissionCoordinator::set_persistence_manager(std::shared_ptr<persistence::PersistenceManager> pm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  persistence_ = std::move(pm);
}

std::shared_ptr<persistence::PersistenceManager> MissionCoordinator::get_persistence_manager() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return persistence_;
}

void MissionCoordinator::handle_target_lock_update(const LiveTargetLock & lock)
{
  std::lock_guard<std::mutex> guard(mutex_);
  // Perception only publishes observations and lock decisions (Property 12).
  // The coordinator evaluates domain rules before deciding if any transition is needed.
  if (lock.is_qualified()) {
    transition_trace_.push_back("LOCK_QUALIFIED: id=" + std::to_string(lock.identity.marker_id));

    // When a qualified live target lock is acquired during DIRECT:
    if (current_strategy_ == flight::StrategyType::DIRECT) {
      transition_trace_.push_back("FLY-008 / EVT_TARGET_ACQUIRED -> PRECISION_LAND.HOVER_BRAKE");
      current_strategy_ = flight::StrategyType::PRECISION_LAND;
      instantiate_precision_land_strategy();
    }
    // When a qualified live target lock is acquired during SEARCH:
    else if (current_strategy_ == flight::StrategyType::SEARCH) {
      std::string eff_policy = search_policy_;
      if (context_ && context_->get_resolved_config() && !context_->get_resolved_config()->routes.search_policy.empty()) {
        eff_policy = context_->get_resolved_config()->routes.search_policy;
      }
      if (eff_policy == "interrupt_on_target") {
        transition_trace_.push_back("FLY-008 / EVT_TARGET_ACQUIRED -> PRECISION_LAND.HOVER_BRAKE");
        current_strategy_ = flight::StrategyType::PRECISION_LAND;
        instantiate_precision_land_strategy();
      } else {
        // complete_grid_first: PadRegistry already records observations, we maintain grid flight
        transition_trace_.push_back("SEARCH_SURVEY_RECORDED: id=" + std::to_string(lock.identity.marker_id));
      }
    }

    if (mode_ && current_strategy_ == flight::StrategyType::PRECISION_LAND) {
      auto * strat = dynamic_cast<flight::PrecisionLandStrategy *>(mode_->current_strategy());
      if (strat) {
        strat->update_target_lock(lock);
      }
    }
  } else if (lock.is_lost()) {
    transition_trace_.push_back("LOCK_LOST");
    if (mode_ && current_strategy_ == flight::StrategyType::PRECISION_LAND) {
      auto * strat = dynamic_cast<flight::PrecisionLandStrategy *>(mode_->current_strategy());
      if (strat) {
        strat->update_target_lock(lock);
      }
    }
  }
}

void MissionCoordinator::handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason reason)
{
  std::lock_guard<std::mutex> guard(mutex_);
  // Do NOT treat normal ground disarm during landing / payload operations as manual takeover
  if (current_strategy_ == flight::StrategyType::LANDED_VERIFIED ||
      current_strategy_ == flight::StrategyType::PAYLOAD_OPERATION ||
      current_strategy_ == flight::StrategyType::TAKEOFF_AFTER_DELIVERY ||
      current_strategy_ == flight::StrategyType::RETURN_LANDED) {
    return;
  }
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

void MissionCoordinator::reset_takeover()
{
  std::lock_guard<std::mutex> guard(mutex_);
  takeover_active_ = false;
  emergency_stop_active_ = false;
  transition_trace_.push_back("TAKEOVER_RESET");
}

void MissionCoordinator::set_custom_direct_target(double lat_deg, double lon_deg, double alt_above_home_m)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_direct_target_.latitude_deg = lat_deg;
  custom_direct_target_.longitude_deg = lon_deg;
  custom_direct_target_.altitude_above_home_m = alt_above_home_m;
  custom_direct_target_.valid = true;
  custom_direct_record_.reset();
}

void MissionCoordinator::set_custom_direct_record(const full_self_driving::msg::PadRecord & record)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_direct_record_ = record;
  custom_direct_target_.valid = false;
}

void MissionCoordinator::reset_custom_direct()
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_direct_target_.valid = false;
  custom_direct_record_.reset();
}

void MissionCoordinator::set_battery_percentage(double pct)
{
  std::lock_guard<std::mutex> guard(mutex_);
  battery_percentage_ = pct;
}

double MissionCoordinator::get_battery_percentage() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return battery_percentage_;
}

void MissionCoordinator::set_direct_enabled(bool enabled)
{
  std::lock_guard<std::mutex> guard(mutex_);
  direct_enabled_ = enabled;
}

bool MissionCoordinator::is_direct_enabled() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return direct_enabled_;
}

void MissionCoordinator::set_trusted_record_max_age_s(double age_s)
{
  std::lock_guard<std::mutex> guard(mutex_);
  trusted_record_max_age_s_ = age_s;
}

double MissionCoordinator::get_trusted_record_max_age_s() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return trusted_record_max_age_s_;
}

void MissionCoordinator::set_minimum_record_quality(float min_q)
{
  std::lock_guard<std::mutex> guard(mutex_);
  minimum_record_quality_ = min_q;
}

float MissionCoordinator::get_minimum_record_quality() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return minimum_record_quality_;
}

void MissionCoordinator::set_max_record_uncertainty_m(double max_unc)
{
  std::lock_guard<std::mutex> guard(mutex_);
  max_record_uncertainty_m_ = max_unc;
}

double MissionCoordinator::get_max_record_uncertainty_m() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return max_record_uncertainty_m_;
}

void MissionCoordinator::set_current_monotonic_ns(uint64_t ns)
{
  std::lock_guard<std::mutex> guard(mutex_);
  current_monotonic_ns_ = ns;
}

uint64_t MissionCoordinator::get_current_monotonic_ns() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return current_monotonic_ns_;
}

bool MissionCoordinator::is_direct_eligible(
  std::string * out_rejection_reason,
  double * out_lat,
  double * out_lon,
  double * out_alt) const
{
  if (!direct_enabled_) {
    if (out_rejection_reason) *out_rejection_reason = "Direct strategy disabled by policy";
    return false;
  }

  // Energy gate
  double min_batt = min_battery_percentage_;
  if (context_ && context_->get_resolved_config()) {
    min_batt = context_->get_resolved_config()->safety.min_battery_percentage;
  }
  if (battery_percentage_ < min_batt) {
    if (out_rejection_reason) {
      *out_rejection_reason = "Insufficient battery energy for Direct (" +
        std::to_string(battery_percentage_) + "% < " + std::to_string(min_batt) + "%)";
    }
    return false;
  }

  // Check custom direct target override
  if (custom_direct_target_.valid) {
    if (!std::isfinite(custom_direct_target_.latitude_deg) || custom_direct_target_.latitude_deg < -90.0 || custom_direct_target_.latitude_deg > 90.0 ||
        !std::isfinite(custom_direct_target_.longitude_deg) || custom_direct_target_.longitude_deg < -180.0 || custom_direct_target_.longitude_deg > 180.0)
    {
      if (out_rejection_reason) *out_rejection_reason = "Custom direct target coordinates invalid or unsafe";
      return false;
    }
    if (out_lat) *out_lat = custom_direct_target_.latitude_deg;
    if (out_lon) *out_lon = custom_direct_target_.longitude_deg;
    if (out_alt) *out_alt = custom_direct_target_.altitude_above_home_m;
    return true;
  }

  uint64_t current_ns = current_monotonic_ns_;
  if (current_ns == 0) {
    current_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  // Check custom direct record override
  if (custom_direct_record_.has_value()) {
    const auto & rec = *custom_direct_record_;
    if (!std::isfinite(rec.latitude_deg) || rec.latitude_deg < -90.0 || rec.latitude_deg > 90.0 ||
        !std::isfinite(rec.longitude_deg) || rec.longitude_deg < -180.0 || rec.longitude_deg > 180.0)
    {
      if (out_rejection_reason) *out_rejection_reason = "Custom direct record coordinates invalid or unsafe";
      return false;
    }
    if (rec.quality < minimum_record_quality_) {
      if (out_rejection_reason) *out_rejection_reason = "Custom direct record quality below threshold";
      return false;
    }
    if (rec.uncertainty_m > max_record_uncertainty_m_) {
      if (out_rejection_reason) *out_rejection_reason = "Custom direct record uncertainty exceeded";
      return false;
    }
    if (rec.last_observed_monotonic_ns > 0) {
      double age_s = (current_ns >= rec.last_observed_monotonic_ns)
        ? static_cast<double>(current_ns - rec.last_observed_monotonic_ns) * 1e-9
        : 0.0;
      if (age_s > trusted_record_max_age_s_) {
        if (out_rejection_reason) *out_rejection_reason = "Custom direct record is stale (age: " + std::to_string(age_s) + " s)";
        return false;
      }
    }
    if (out_lat) *out_lat = rec.latitude_deg;
    if (out_lon) *out_lon = rec.longitude_deg;
    double direct_alt = 15.0;
    if (context_ && context_->get_resolved_config()) {
      direct_alt = context_->get_resolved_config()->routes.search_altitude_m;
    }
    if (out_alt) *out_alt = direct_alt;
    return true;
  }

  // Check PadRegistry lookup
  if (!pad_registry_) {
    if (out_rejection_reason) *out_rejection_reason = "PadRegistry not configured";
    return false;
  }

  if (!context_) {
    if (out_rejection_reason) *out_rejection_reason = "MissionContext not available";
    return false;
  }

  const auto & sel = context_->get_selection();
  if (!sel.target.has_value()) {
    if (out_rejection_reason) *out_rejection_reason = "No target identity selected";
    return false;
  }

  auto record_opt = pad_registry_->lookup(*sel.target, sel.map_id, sel.scenario_id);
  if (!record_opt.has_value()) {
    if (out_rejection_reason) {
      *out_rejection_reason = "No trusted pad record matching target and scope (" +
        sel.map_id + "/" + sel.scenario_id + ")";
    }
    return false;
  }

  const auto & record = *record_opt;

  // Target Identity check
  if (record.identity.marker_id != sel.target->marker_id)
  {
    if (out_rejection_reason) *out_rejection_reason = "Pad record marker ID mismatch";
    return false;
  }

  if (!sel.target->dictionary.empty() && !record.identity.dictionary.empty() &&
      record.identity.dictionary != sel.target->dictionary)
  {
    if (out_rejection_reason) *out_rejection_reason = "Pad record dictionary mismatch";
    return false;
  }

  // Quality check
  if (record.quality < minimum_record_quality_) {
    if (out_rejection_reason) {
      *out_rejection_reason = "Pad record quality below threshold (" +
        std::to_string(record.quality) + " < " + std::to_string(minimum_record_quality_) + ")";
    }
    return false;
  }

  // Uncertainty check
  if (record.uncertainty_m > max_record_uncertainty_m_) {
    if (out_rejection_reason) {
      *out_rejection_reason = "Pad record position uncertainty exceeded (" +
        std::to_string(record.uncertainty_m) + " > " + std::to_string(max_record_uncertainty_m_) + ")";
    }
    return false;
  }

  // Age / Freshness check
  if (record.last_observed_monotonic_ns > 0) {
    double age_s = (current_ns >= record.last_observed_monotonic_ns)
      ? static_cast<double>(current_ns - record.last_observed_monotonic_ns) * 1e-9
      : 0.0;
    if (age_s > trusted_record_max_age_s_) {
      if (out_rejection_reason) {
        *out_rejection_reason = "Pad record is stale (age: " + std::to_string(age_s) + " s > " +
          std::to_string(trusted_record_max_age_s_) + " s)";
      }
      return false;
    }
  }

  // Path / coordinates check
  if (!std::isfinite(record.latitude_deg) || record.latitude_deg < -90.0 || record.latitude_deg > 90.0 ||
      !std::isfinite(record.longitude_deg) || record.longitude_deg < -180.0 || record.longitude_deg > 180.0)
  {
    if (out_rejection_reason) *out_rejection_reason = "Pad record coordinates invalid or unsafe";
    return false;
  }

  if (out_lat) *out_lat = record.latitude_deg;
  if (out_lon) *out_lon = record.longitude_deg;
  double direct_alt = 15.0;
  if (context_ && context_->get_resolved_config()) {
    direct_alt = context_->get_resolved_config()->routes.search_altitude_m;
  }
  if (out_alt) *out_alt = direct_alt;

  return true;
}

bool MissionCoordinator::is_search_plan_valid() const
{
  if (has_custom_search_plan_) {
    return !custom_search_plan_.get_source_route().waypoints.empty();
  }
  if (has_custom_search_route_) {
    return !custom_search_route_.waypoints.empty();
  }
  if (plan_manager_) {
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
    if (wp && !wp->get_source_route().waypoints.empty()) {
      return true;
    }
    // PlanManager configured but has no matching working plan
    return false;
  }
  return true;
}

void MissionCoordinator::instantiate_direct_strategy(double lat, double lon, double alt)
{
  if (!mode_) return;
  float arrival_rad = 4.0f;
  float max_h_speed = 5.0f;
  float max_yaw_rate = 0.785398163f;

  if (context_ && context_->get_resolved_config()) {
    const auto & cfg = context_->get_resolved_config()->routes;
    arrival_rad = static_cast<float>(cfg.acceptance_radius_m);
    max_h_speed = static_cast<float>(cfg.transit_in_speed_m_s);
    max_yaw_rate = static_cast<float>(cfg.max_yaw_rate_deg_s * M_PI / 180.0);
  }

  mode_->set_strategy(std::make_unique<flight::DirectStrategy>(
    mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(),
    lat, lon, alt, arrival_rad, 0.5f, 1.0f, max_h_speed, max_yaw_rate, 120.0));
}

void MissionCoordinator::instantiate_search_strategy()
{
  if (!mode_) return;
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

void MissionCoordinator::instantiate_precision_land_strategy()
{
  if (!mode_) return;
  float max_vel = 1.5f;
  float descent_vel = 0.35f;
  float p_gain = 0.9f;
  float i_gain = 0.0f;
  float target_timeout = 3.0f;
  float delta_pos = 0.25f;
  float delta_vel = 0.25f;
  float stabilize_duration = 1.0f;
  double search_alt = 15.0;
  double approach_alt = 2.5;

  if (context_ && context_->get_resolved_config()) {
    const auto & cfg = context_->get_resolved_config()->routes;
    const auto & safety = context_->get_resolved_config()->safety;
    approach_alt = cfg.approach_altitude_m;
    search_alt = cfg.search_altitude_m;
    if (cfg.landing_descent_rate_m_s > 0.0) {
      descent_vel = static_cast<float>(cfg.landing_descent_rate_m_s);
    }
    if (safety.target_loss_timeout_s > 0.0) {
      target_timeout = static_cast<float>(safety.target_loss_timeout_s);
    }
  }

  mode_->set_strategy(std::make_unique<flight::PrecisionLandStrategy>(
    mode_->node(), mode_->goto_global_setpoint(), mode_->trajectory_setpoint(), mode_->state_cache(),
    max_vel, descent_vel, p_gain, i_gain, target_timeout,
    delta_pos, delta_vel, stabilize_duration, search_alt, approach_alt));
}

void MissionCoordinator::set_search_policy(const std::string & policy)
{
  std::lock_guard<std::mutex> guard(mutex_);
  search_policy_ = policy;
}

std::string MissionCoordinator::get_search_policy() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return search_policy_;
}

bool MissionCoordinator::handle_search_completed()
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (current_strategy_ != flight::StrategyType::SEARCH) {
    return false;
  }

  transition_trace_.push_back("SEARCH_GRID_COMPLETED");

  std::string rejection_reason;
  double direct_lat = 0.0, direct_lon = 0.0, direct_alt = 15.0;
  bool direct_ok = is_direct_eligible(&rejection_reason, &direct_lat, &direct_lon, &direct_alt);

  if (direct_ok) {
    transition_trace_.push_back("FLY-004 / EVT_SEARCH_TO_DIRECT_TRANSITION (lat=" +
      std::to_string(direct_lat) + ", lon=" + std::to_string(direct_lon) + ")");
    current_strategy_ = flight::StrategyType::DIRECT;
    instantiate_direct_strategy(direct_lat, direct_lon, direct_alt);
    return true;
  } else {
    transition_trace_.push_back("SURVEY_COMPLETE_TARGET_MISSING: " + rejection_reason + " -> TRANSIT_OUT");
    current_strategy_ = flight::StrategyType::TRANSIT_OUT;
    if (mode_) {
      Route route = resolve_transit_out_route();
      mode_->set_strategy(std::make_unique<flight::TransitOutStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route, persistence_));
    }
    return true;
  }
}

bool MissionCoordinator::handle_direct_complete()
{
  return request_transition(flight::StrategyType::PRECISION_LAND);
}

bool MissionCoordinator::handle_direct_fallback(const std::string & reason)
{
  (void)reason;
  std::string err;
  return request_transition(flight::StrategyType::SEARCH, &err);
}

bool MissionCoordinator::handle_landing_verified()
{
  std::string err;
  if (!request_transition(flight::StrategyType::LANDED_VERIFIED, &err)) {
    return false;
  }
  return request_transition(flight::StrategyType::PAYLOAD_OPERATION, &err);
}

bool MissionCoordinator::handle_payload_complete(uint8_t result)
{
  std::string err;
  if (result == full_self_driving::msg::PayloadStatus::RESULT_SUCCESS) {
    return request_transition(flight::StrategyType::TAKEOFF_AFTER_DELIVERY, &err);
  } else {
    return request_transition(flight::StrategyType::RETURN_STRATEGY, &err);
  }
}

void MissionCoordinator::instantiate_payload_operation_strategy()
{
  if (!mode_) return;
  auto strat = std::make_unique<flight::PayloadOperationStrategy>(
    mode_->node(), payload_controller_, persistence_, context_);
  strat->on_enter();
  mode_->set_strategy(std::move(strat));
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

  // Branch evaluation for ACQUIRE_TARGET
  if (next_strategy == flight::StrategyType::ACQUIRE_TARGET) {
    std::string direct_rejection_reason;
    double direct_lat = 0.0, direct_lon = 0.0, direct_alt = 15.0;
    bool direct_ok = is_direct_eligible(&direct_rejection_reason, &direct_lat, &direct_lon, &direct_alt);

    if (mode_) {
      uint32_t tid = (context_ && context_->get_selection().target.has_value())
        ? context_->get_selection().target->marker_id : 0;
      if (direct_ok) {
        RCLCPP_INFO(mode_->node().get_logger(),
          "[COORDINATOR] Direct Acquisition ELIGIBLE for Target ID %u! Destination: lat=%.7f, lon=%.7f, alt=%.2f m",
          tid, direct_lat, direct_lon, direct_alt);
      } else {
        RCLCPP_INFO(mode_->node().get_logger(),
          "[COORDINATOR] Direct Acquisition INELIGIBLE for Target ID %u: %s. Branching to Search.",
          tid, direct_rejection_reason.c_str());
      }
    }

    if (direct_ok) {
      transition_trace_.push_back("FLY-004 / EVT_ACQUISITION_DIRECT_SELECTED (lat=" +
        std::to_string(direct_lat) + ", lon=" + std::to_string(direct_lon) + ")");
      current_strategy_ = flight::StrategyType::DIRECT;
      instantiate_direct_strategy(direct_lat, direct_lon, direct_alt);
      return true;
    } else {
      bool search_ok = is_search_plan_valid();
      if (search_ok) {
        transition_trace_.push_back("FLY-005 / EVT_ACQUISITION_SEARCH_SELECTED (" + direct_rejection_reason + ")");
        current_strategy_ = flight::StrategyType::SEARCH;
        instantiate_search_strategy();
        return true;
      } else {
        transition_trace_.push_back("ACQUISITION_FAILED_HOLD: " + direct_rejection_reason +
          "; no valid working plan for search fallback");
        current_strategy_ = flight::StrategyType::HOLD;
        if (out_error) {
          *out_error = "Direct acquisition ineligible (" + direct_rejection_reason +
            ") and Search fallback unavailable (no valid working plan)";
        }
        return false;
      }
    }
  }

  // Transition from DIRECT to PRECISION_LAND (Direct completion)
  if (current_strategy_ == flight::StrategyType::DIRECT && next_strategy == flight::StrategyType::PRECISION_LAND) {
    transition_trace_.push_back("FLY-006 / EVT_DIRECT_COMPLETE -> PRECISION_LAND.SEARCH");
    current_strategy_ = flight::StrategyType::PRECISION_LAND;
    instantiate_precision_land_strategy();
    return true;
  }

  // Transition from SEARCH to DIRECT (Survey completion transition)
  if (current_strategy_ == flight::StrategyType::SEARCH && next_strategy == flight::StrategyType::DIRECT) {
    std::string direct_rejection_reason;
    double direct_lat = 0.0, direct_lon = 0.0, direct_alt = 15.0;
    bool direct_ok = is_direct_eligible(&direct_rejection_reason, &direct_lat, &direct_lon, &direct_alt);
    if (direct_ok) {
      transition_trace_.push_back("FLY-004 / EVT_SEARCH_TO_DIRECT_TRANSITION (lat=" +
        std::to_string(direct_lat) + ", lon=" + std::to_string(direct_lon) + ")");
      current_strategy_ = flight::StrategyType::DIRECT;
      instantiate_direct_strategy(direct_lat, direct_lon, direct_alt);
      return true;
    } else {
      if (out_error) *out_error = "Cannot transition SEARCH to DIRECT: " + direct_rejection_reason;
      return false;
    }
  }

  // Transition from DIRECT to SEARCH (Direct fallback during flight)
  if (current_strategy_ == flight::StrategyType::DIRECT && next_strategy == flight::StrategyType::SEARCH) {
    if (is_search_plan_valid()) {
      transition_trace_.push_back("FLY-007 / EVT_DIRECT_FALLBACK -> SEARCH");
      current_strategy_ = flight::StrategyType::SEARCH;
      instantiate_search_strategy();
      return true;
    } else {
      transition_trace_.push_back("DIRECT_FALLBACK_HOLD: no valid working plan for search");
      current_strategy_ = flight::StrategyType::HOLD;
      if (out_error) {
        *out_error = "Direct fallback failed: no valid working plan for search";
      }
      return false;
    }
  }

  // Transition from PRECISION_LAND to LANDED_VERIFIED
  if (current_strategy_ == flight::StrategyType::PRECISION_LAND && next_strategy == flight::StrategyType::LANDED_VERIFIED) {
    transition_trace_.push_back("FLY-012 / EVT_LANDING_VERIFIED -> LANDED_VERIFIED");
    current_strategy_ = flight::StrategyType::LANDED_VERIFIED;
    return true;
  }

  // Transition from LANDED_VERIFIED to PAYLOAD_OPERATION
  if (current_strategy_ == flight::StrategyType::LANDED_VERIFIED && next_strategy == flight::StrategyType::PAYLOAD_OPERATION) {
    transition_trace_.push_back("FLY-013 / EVT_PAYLOAD_GATES_PASSED -> PAYLOAD_OPERATION");
    current_strategy_ = flight::StrategyType::PAYLOAD_OPERATION;
    if (mode_) {
      instantiate_payload_operation_strategy();
    }
    return true;
  }

  // Transition from PAYLOAD_OPERATION to TAKEOFF_AFTER_DELIVERY
  if (current_strategy_ == flight::StrategyType::PAYLOAD_OPERATION && next_strategy == flight::StrategyType::TAKEOFF_AFTER_DELIVERY) {
    transition_trace_.push_back("FLY-014 / EVT_PAYLOAD_SUCCESS -> TAKEOFF_AFTER_DELIVERY");
    current_strategy_ = flight::StrategyType::TAKEOFF_AFTER_DELIVERY;
    if (mode_) {
      double takeoff_alt = 20.0;
      if (context_ && context_->get_resolved_config()) {
        takeoff_alt = context_->get_resolved_config()->routes.transit_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::TakeoffStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), takeoff_alt,
        1.0, 0.5, 30.0, flight::StrategyType::TAKEOFF_AFTER_DELIVERY));
    }
    return true;
  }

  // Transition from PAYLOAD_OPERATION to RETURN_STRATEGY (on failure or unknown)
  if (current_strategy_ == flight::StrategyType::PAYLOAD_OPERATION && next_strategy == flight::StrategyType::RETURN_STRATEGY) {
    transition_trace_.push_back("FLY-015 / EVT_PAYLOAD_NON_SUCCESS -> RETURN_STRATEGY");
    current_strategy_ = flight::StrategyType::RETURN_STRATEGY;
    return true;
  }

  // Transition from TAKEOFF_AFTER_DELIVERY to TRANSIT_OUT
  if (current_strategy_ == flight::StrategyType::TAKEOFF_AFTER_DELIVERY && next_strategy == flight::StrategyType::TRANSIT_OUT) {
    transition_trace_.push_back("FLY-017 / EVT_TAKEOFF_AFTER_DELIVERY_COMPLETE -> TRANSIT_OUT");
    current_strategy_ = flight::StrategyType::TRANSIT_OUT;
    if (mode_) {
      Route route;
      if (has_custom_transit_out_route_) {
        route = custom_transit_out_route_;
      } else {
        route = Route::create_default_kmitl_transit_out_route();
        if (context_ && context_->get_resolved_config()) {
          const auto & cfg = context_->get_resolved_config()->routes;
          route.set_max_horizontal_speed_m_s(static_cast<float>(cfg.transit_out_speed_m_s));
          route.set_transit_altitude_above_home_m(cfg.transit_altitude_m);
          route.set_acceptance_radius_m(static_cast<float>(cfg.acceptance_radius_m));
          route.set_max_yaw_rate_deg_s(static_cast<float>(cfg.max_yaw_rate_deg_s));
        }
      }
      mode_->set_strategy(std::make_unique<flight::TransitOutStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route, persistence_));
    }
    return true;
  }

  // Transition from TRANSIT_OUT to RETURN_STRATEGY
  if (current_strategy_ == flight::StrategyType::TRANSIT_OUT && next_strategy == flight::StrategyType::RETURN_STRATEGY) {
    transition_trace_.push_back("FLY-018 / EVT_TRANSIT_OUT_COMPLETE -> RETURN_STRATEGY");
    current_strategy_ = flight::StrategyType::RETURN_STRATEGY;
    if (mode_) {
      double return_alt = 20.0;
      if (context_ && context_->get_resolved_config()) {
        return_alt = context_->get_resolved_config()->routes.transit_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::ReturnStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->trajectory_setpoint(),
        mode_->state_cache(), persistence_, context_,
        flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME, return_alt));
    }
    return true;
  }

  // Transition from RETURN_STRATEGY to RETURN_LANDED
  if (current_strategy_ == flight::StrategyType::RETURN_STRATEGY && next_strategy == flight::StrategyType::RETURN_LANDED) {
    transition_trace_.push_back("FLY-019 / EVT_RETURN_ARRIVED -> RETURN_LANDED");
    current_strategy_ = flight::StrategyType::RETURN_LANDED;
    return true;
  }

  std::string trace_entry = flight::strategy_type_to_string(current_strategy_) +
    " -> " + flight::strategy_type_to_string(next_strategy);
  transition_trace_.push_back(trace_entry);

  current_strategy_ = next_strategy;

  if (mode_) {
    if (next_strategy == flight::StrategyType::WAITING_FOR_MODE) {
      mode_->set_strategy(std::make_unique<flight::WaitingForModeStrategy>());
    } else if (next_strategy == flight::StrategyType::TAKEOFF) {
      double takeoff_alt = 20.0;
      if (context_ && context_->get_resolved_config()) {
        takeoff_alt = context_->get_resolved_config()->routes.transit_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::TakeoffStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), takeoff_alt));
    } else if (next_strategy == flight::StrategyType::TRANSIT_IN) {
      Route route = resolve_transit_in_route();
      mode_->set_strategy(std::make_unique<flight::TransitInStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route));
    } else if (next_strategy == flight::StrategyType::DIRECT) {
      double direct_lat = 13.73132845, direct_lon = 100.78990948, direct_alt = 5.0;
      is_direct_eligible(nullptr, &direct_lat, &direct_lon, &direct_alt);
      instantiate_direct_strategy(direct_lat, direct_lon, direct_alt);
    } else if (next_strategy == flight::StrategyType::SEARCH) {
      instantiate_search_strategy();
    } else if (next_strategy == flight::StrategyType::PRECISION_LAND) {
      instantiate_precision_land_strategy();
    } else if (next_strategy == flight::StrategyType::PAYLOAD_OPERATION) {
      instantiate_payload_operation_strategy();
    } else if (next_strategy == flight::StrategyType::TAKEOFF_AFTER_DELIVERY) {
      double takeoff_alt = 20.0;
      if (context_ && context_->get_resolved_config()) {
        takeoff_alt = context_->get_resolved_config()->routes.transit_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::TakeoffStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), takeoff_alt,
        1.0, 0.5, 30.0, flight::StrategyType::TAKEOFF_AFTER_DELIVERY));
    } else if (next_strategy == flight::StrategyType::TRANSIT_OUT) {
      Route route = resolve_transit_out_route();
      mode_->set_strategy(std::make_unique<flight::TransitOutStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route, persistence_));
    } else if (next_strategy == flight::StrategyType::RETURN_STRATEGY) {
      double return_alt = 20.0;
      if (context_ && context_->get_resolved_config()) {
        return_alt = context_->get_resolved_config()->routes.transit_altitude_m;
      }
      mode_->set_strategy(std::make_unique<flight::ReturnStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->trajectory_setpoint(),
        mode_->state_cache(), persistence_, context_,
        flight::ReturnStrategy::ReturnMode::RETURN_TO_HOME, return_alt));
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

void MissionCoordinator::set_custom_transit_out_route(const Route & route)
{
  std::lock_guard<std::mutex> guard(mutex_);
  custom_transit_out_route_ = route;
  has_custom_transit_out_route_ = true;
}

void MissionCoordinator::reset_custom_transit_out_route()
{
  std::lock_guard<std::mutex> guard(mutex_);
  has_custom_transit_out_route_ = false;
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

Route MissionCoordinator::resolve_transit_in_route() const
{
  Route route;
  std::optional<WorkingPlan> active_wp;
  if (plan_manager_ && context_) {
    const auto & sel = context_->get_selection();
    active_wp = plan_manager_->get_working_plan(sel.working_plan_id);
    if (!active_wp) {
      active_wp = plan_manager_->get_active_working_plan(sel.map_id, sel.scenario_id);
    }
  }

  if (active_wp && active_wp->has_transit_in_route()) {
    route = active_wp->get_transit_in_route();
  } else if (has_custom_transit_in_route_) {
    route = custom_transit_in_route_;
  } else {
    route = Route::create_default_kmitl_transit_in_route();
  }

  if (context_ && context_->get_resolved_config()) {
    const auto & cfg = context_->get_resolved_config()->routes;
    route.set_max_horizontal_speed_m_s(static_cast<float>(cfg.transit_in_speed_m_s));
    route.set_transit_altitude_above_home_m(cfg.transit_altitude_m);
    route.set_acceptance_radius_m(static_cast<float>(cfg.acceptance_radius_m));
    route.set_max_yaw_rate_deg_s(static_cast<float>(cfg.max_yaw_rate_deg_s));
  }
  return route;
}

Route MissionCoordinator::resolve_transit_out_route() const
{
  Route route;
  std::optional<WorkingPlan> active_wp;
  if (plan_manager_ && context_) {
    const auto & sel = context_->get_selection();
    active_wp = plan_manager_->get_working_plan(sel.working_plan_id);
    if (!active_wp) {
      active_wp = plan_manager_->get_active_working_plan(sel.map_id, sel.scenario_id);
    }
  }

  if (active_wp && active_wp->has_transit_out_route()) {
    route = active_wp->get_transit_out_route();
  } else if (has_custom_transit_out_route_) {
    route = custom_transit_out_route_;
  } else {
    route = Route::create_default_kmitl_transit_out_route();
  }

  if (context_ && context_->get_resolved_config()) {
    const auto & cfg = context_->get_resolved_config()->routes;
    route.set_max_horizontal_speed_m_s(static_cast<float>(cfg.transit_out_speed_m_s));
    route.set_transit_altitude_above_home_m(cfg.transit_altitude_m);
    route.set_acceptance_radius_m(static_cast<float>(cfg.acceptance_radius_m));
    route.set_max_yaw_rate_deg_s(static_cast<float>(cfg.max_yaw_rate_deg_s));
  }
  return route;
}

}  // namespace full_self_driving::domain
