#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "domain/mission_context.hpp"
#include "domain/live_target_lock.hpp"
#include "domain/route.hpp"
#include "domain/working_plan.hpp"
#include "flight/internal_strategy.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "runtime/plan_manager.hpp"

namespace full_self_driving::domain
{

class MissionCoordinator
{
public:
  explicit MissionCoordinator(std::shared_ptr<MissionContext> context);
  ~MissionCoordinator() = default;

  void bind_executor(std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor,
                     std::shared_ptr<flight::FullSelfDrivingMode> mode);

  void set_plan_manager(std::shared_ptr<runtime::PlanManager> pm);
  std::shared_ptr<runtime::PlanManager> get_plan_manager() const;

  void handle_target_lock_update(const LiveTargetLock & lock);
  void handle_takeover(flight::FullSelfDrivingModeExecutor::DeactivateReason reason);
  void handle_emergency_stop();

  bool request_transition(flight::StrategyType next_strategy, std::string * out_error = nullptr);

  flight::StrategyType get_current_strategy() const { return current_strategy_; }
  uint8_t get_flight_phase() const;
  bool is_takeover_active() const { return takeover_active_; }
  bool is_emergency_stop_active() const { return emergency_stop_active_; }

  const std::vector<std::string> & get_transition_trace() const { return transition_trace_; }
  void clear_transition_trace();

  void set_custom_transit_in_route(const Route & route);
  void reset_custom_transit_in_route();

  void set_custom_search_route(const CanonicalSearchRoute & route);
  void set_custom_search_plan(const WorkingPlan & wp);
  void reset_custom_search();

private:
  mutable std::mutex mutex_;
  std::shared_ptr<MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;

  flight::StrategyType current_strategy_{flight::StrategyType::WAITING_FOR_MODE};
  bool takeover_active_{false};
  bool emergency_stop_active_{false};

  Route custom_transit_in_route_;
  bool has_custom_transit_in_route_{false};

  WorkingPlan custom_search_plan_;
  CanonicalSearchRoute custom_search_route_;
  bool has_custom_search_plan_{false};
  bool has_custom_search_route_{false};

  std::vector<std::string> transition_trace_;
};

}  // namespace full_self_driving::domain
