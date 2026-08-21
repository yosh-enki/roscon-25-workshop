#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "domain/mission_context.hpp"
#include "domain/live_target_lock.hpp"
#include "domain/route.hpp"
#include "domain/working_plan.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "flight/strategies/direct_strategy.hpp"
#include "flight/strategies/precision_land_strategy.hpp"
#include "flight/strategies/payload_operation_strategy.hpp"
#include "flight/strategies/transit_out_strategy.hpp"
#include "flight/strategies/return_strategy.hpp"
#include "payload/payload_controller.hpp"
#include "persistence/persistence_manager.hpp"
#include "full_self_driving/msg/pad_record.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"

namespace full_self_driving::domain
{

struct CustomDirectTarget
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_above_home_m{5.0};
  bool valid{false};
};

class MissionCoordinator
{
public:
  explicit MissionCoordinator(std::shared_ptr<MissionContext> context);
  ~MissionCoordinator() = default;

  void bind_executor(std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor,
                     std::shared_ptr<flight::FullSelfDrivingMode> mode);

  void set_plan_manager(std::shared_ptr<runtime::PlanManager> pm);
  std::shared_ptr<runtime::PlanManager> get_plan_manager() const;

  void set_pad_registry(std::shared_ptr<registry::PadRegistry> registry);
  std::shared_ptr<registry::PadRegistry> get_pad_registry() const;

  void set_payload_controller(std::shared_ptr<payload::PayloadController> pc);
  std::shared_ptr<payload::PayloadController> get_payload_controller() const;

  void set_persistence_manager(std::shared_ptr<persistence::PersistenceManager> pm);
  std::shared_ptr<persistence::PersistenceManager> get_persistence_manager() const;

  void handle_target_lock_update(const LiveTargetLock & lock);
  void handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason reason);
  void handle_emergency_stop();
  void reset_takeover();

  bool request_transition(flight::StrategyType next_strategy, std::string * out_error = nullptr);

  flight::StrategyType get_current_strategy() const { return current_strategy_; }
  uint8_t get_flight_phase() const;
  bool is_takeover_active() const { return takeover_active_; }
  bool is_emergency_stop_active() const { return emergency_stop_active_; }

  const std::vector<std::string> & get_transition_trace() const { return transition_trace_; }
  void clear_transition_trace();

  void set_custom_transit_in_route(const Route & route);
  void reset_custom_transit_in_route();

  void set_custom_transit_out_route(const Route & route);
  void reset_custom_transit_out_route();

  void set_custom_search_route(const CanonicalSearchRoute & route);
  void set_custom_search_plan(const WorkingPlan & wp);
  void reset_custom_search();

  // Direct strategy configuration and custom testing hooks
  void set_custom_direct_target(double lat_deg, double lon_deg, double alt_above_home_m = 5.0);
  void set_custom_direct_record(const full_self_driving::msg::PadRecord & record);
  void reset_custom_direct();

  void set_battery_percentage(double pct);
  double get_battery_percentage() const;

  void set_direct_enabled(bool enabled);
  bool is_direct_enabled() const;

  void set_trusted_record_max_age_s(double age_s);
  double get_trusted_record_max_age_s() const;

  void set_minimum_record_quality(float min_q);
  float get_minimum_record_quality() const;

  void set_max_record_uncertainty_m(double max_unc);
  double get_max_record_uncertainty_m() const;

  void set_current_monotonic_ns(uint64_t ns);
  uint64_t get_current_monotonic_ns() const;

  // Direct fallback and completion helpers
  bool handle_direct_complete();
  bool handle_direct_fallback(const std::string & reason = "direct navigation timed out");

  // Landing verified and payload helpers
  bool handle_landing_verified();
  bool handle_payload_complete(uint8_t result);

  // Invariant query
  bool is_direct_eligible(
    std::string * out_rejection_reason = nullptr,
    double * out_lat = nullptr,
    double * out_lon = nullptr,
    double * out_alt = nullptr) const;

  bool is_search_plan_valid() const;

private:
  void instantiate_direct_strategy(double lat, double lon, double alt);
  void instantiate_search_strategy();
  void instantiate_precision_land_strategy();
  void instantiate_payload_operation_strategy();

  mutable std::mutex mutex_;
  std::shared_ptr<MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> pad_registry_;
  std::shared_ptr<payload::PayloadController> payload_controller_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;

  flight::StrategyType current_strategy_{flight::StrategyType::WAITING_FOR_MODE};
  bool takeover_active_{false};
  bool emergency_stop_active_{false};

  Route custom_transit_in_route_;
  bool has_custom_transit_in_route_{false};

  Route custom_transit_out_route_;
  bool has_custom_transit_out_route_{false};

  WorkingPlan custom_search_plan_;
  CanonicalSearchRoute custom_search_route_;
  bool has_custom_search_plan_{false};
  bool has_custom_search_route_{false};

  CustomDirectTarget custom_direct_target_;
  std::optional<full_self_driving::msg::PadRecord> custom_direct_record_;

  bool direct_enabled_{true};
  double battery_percentage_{100.0};
  double min_battery_percentage_{20.0};
  double trusted_record_max_age_s_{3600.0};
  float minimum_record_quality_{0.0f};
  double max_record_uncertainty_m_{50.0};
  uint64_t current_monotonic_ns_{0};

  std::vector<std::string> transition_trace_;
};

}  // namespace full_self_driving::domain
