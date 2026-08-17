#include "domain/mission_coordinator.hpp"

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
    // In later tasks specific strategy instances will be attached.
    // For now we set WaitingForModeStrategy or placeholder
    if (next_strategy == flight::StrategyType::WAITING_FOR_MODE) {
      mode_->set_strategy(std::make_unique<flight::WaitingForModeStrategy>());
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

}  // namespace full_self_driving::domain
