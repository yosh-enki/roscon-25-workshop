#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "full_self_driving/msg/full_self_driving_state.hpp"
#include "full_self_driving/msg/readiness_report.hpp"
#include "full_self_driving/msg/flight_safety_status.hpp"
#include "full_self_driving/msg/vehicle_telemetry.hpp"
#include "full_self_driving/msg/live_target_lock.hpp"
#include "full_self_driving/srv/emergency_stop.hpp"

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/lifecycle_supervisor.hpp"
#include "adapters/px4_api_capabilities.hpp"
#include "adapters/px4_state_cache.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"

namespace full_self_driving::runtime
{

class FlightRuntimeNode : public rclcpp::Node
{
public:
  explicit FlightRuntimeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FlightRuntimeNode() override = default;

  bool is_mode_registered() const { return mode_registered_; }
  bool is_ready_for_ownmode() const;

  std::shared_ptr<domain::MissionContext> mission_context() const { return context_; }
  std::shared_ptr<domain::MissionCoordinator> coordinator() const { return coordinator_; }
  std::shared_ptr<LifecycleSupervisor> supervisor() const { return supervisor_; }
  std::shared_ptr<persistence::PersistenceManager> persistence() const { return persistence_; }
  std::shared_ptr<flight::FullSelfDrivingMode> mode() const { return mode_; }
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor() const { return executor_; }

  void trigger_evaluation_cycle();

private:
  void initialize_components();
  void check_and_register_mode();
  void publish_status_cycle();

  // Publishers
  rclcpp::Publisher<full_self_driving::msg::FullSelfDrivingState>::SharedPtr state_pub_;
  rclcpp::Publisher<full_self_driving::msg::ReadinessReport>::SharedPtr readiness_pub_;
  rclcpp::Publisher<full_self_driving::msg::FlightSafetyStatus>::SharedPtr safety_pub_;
  rclcpp::Publisher<full_self_driving::msg::VehicleTelemetry>::SharedPtr telemetry_pub_;

  // Subscriptions & Services
  rclcpp::Subscription<full_self_driving::msg::LiveTargetLock>::SharedPtr target_lock_sub_;
  rclcpp::Service<full_self_driving::srv::EmergencyStop>::SharedPtr emergency_stop_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr periodic_timer_;

  // Domain, State & Lifecycle
  std::shared_ptr<domain::EngineeringConfig> config_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<LifecycleSupervisor> supervisor_;
  std::unique_ptr<px4_ros2::Context> px4_context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;

  // Single Mode and Single Executor
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;

  bool mode_registered_{false};
  bool api_manifest_valid_{false};
  std::string config_path_;
  std::string manifest_path_;
  uint64_t state_sequence_{0};
};

}  // namespace full_self_driving::runtime
