#include "runtime/flight_runtime_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <cmath>

namespace full_self_driving::runtime
{

using namespace std::chrono_literals;

FlightRuntimeNode::FlightRuntimeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fsd_flight_runtime", options)
{
  this->declare_parameter<std::string>("engineering_config", "");
  this->declare_parameter<std::string>("manifest_path", "");
  this->declare_parameter<bool>("simulation", true);
  this->declare_parameter<std::string>("world", "kmitl_airfield");
  this->declare_parameter<std::string>("flight_fixture", "none");

  config_path_ = this->get_parameter("engineering_config").as_string();
  manifest_path_ = this->get_parameter("manifest_path").as_string();

  if (manifest_path_.empty()) {
    try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
      manifest_path_ = share_dir + "/config/pinned_api_manifest.yaml";
    } catch (...) {
      manifest_path_ = "config/pinned_api_manifest.yaml";
    }
  }

  initialize_components();
}

void FlightRuntimeNode::initialize_components()
{
  RCLCPP_INFO(get_logger(), "[RUNTIME] Initializing FlightRuntimeNode...");

  // Publishers
  state_pub_ = this->create_publisher<full_self_driving::msg::FullSelfDrivingState>(
    "/full_self_driving/state", 10);
  readiness_pub_ = this->create_publisher<full_self_driving::msg::ReadinessReport>(
    "/full_self_driving/readiness", 10);
  safety_pub_ = this->create_publisher<full_self_driving::msg::FlightSafetyStatus>(
    "/full_self_driving/safety", 10);
  telemetry_pub_ = this->create_publisher<full_self_driving::msg::VehicleTelemetry>(
    "/full_self_driving/telemetry", 10);
  working_plan_status_pub_ = this->create_publisher<full_self_driving::msg::WorkingPlanStatus>(
    "/full_self_driving/working_plan/status", 10);

  // Target Lock Subscription
  target_lock_sub_ = this->create_subscription<full_self_driving::msg::LiveTargetLock>(
    "/full_self_driving/perception/target_lock", 10,
    [this](full_self_driving::msg::LiveTargetLock::ConstSharedPtr msg) {
      if (coordinator_) {
        coordinator_->handle_target_lock_update(domain::LiveTargetLock::from_msg(*msg));
      }
    });

  // Emergency Stop Service
  emergency_stop_srv_ = this->create_service<full_self_driving::srv::EmergencyStop>(
    "/full_self_driving/emergency_stop",
    [this](
      const std::shared_ptr<full_self_driving::srv::EmergencyStop::Request> req,
      std::shared_ptr<full_self_driving::srv::EmergencyStop::Response> res)
    {
      (void)req;
      RCLCPP_WARN(get_logger(), "[RUNTIME] Emergency stop service invoked!");
      if (coordinator_) {
        coordinator_->handle_emergency_stop();
      }
      res->success = true;
      res->message = "Emergency stop applied by coordinator";
      res->stop_monotonic_ns = this->get_clock()->now().nanoseconds();
    });

  // Verify API Manifest
  auto api_report = adapters::Px4ApiCapabilities::verify_api_manifest(manifest_path_);
  api_manifest_valid_ = api_report.is_valid;
  if (!api_manifest_valid_) {
    RCLCPP_ERROR(get_logger(), "[RUNTIME] Pinned API manifest validation failed: %s",
      api_report.to_string().c_str());
  } else {
    RCLCPP_INFO(get_logger(), "[RUNTIME] Pinned API manifest verified successfully");
  }

  // Load Engineering Config
  if (!config_path_.empty()) {
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::from_yaml_file(config_path_));
  } else {
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
  }

  // Initialize Plan Manager
  plan_manager_ = std::make_shared<PlanManager>("/tmp/fsd_plans");

  // Load default plan fixture if available
  std::string default_plan_path;
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
    std::vector<std::string> candidates = {
      share_dir + "/test/fixtures/plans/aavc2026_mission.plan",
      "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/test/fixtures/plans/aavc2026_mission.plan",
      "/home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/plans/aavc2026_mission.plan"
    };
    for (const auto & c : candidates) {
      if (std::filesystem::is_regular_file(c)) {
        default_plan_path = c;
        break;
      }
    }
  } catch (...) {}

  std::string default_artifact_id;
  std::string default_wp_id;
  if (!default_plan_path.empty()) {
    std::ifstream file(default_plan_path, std::ios::binary);
    if (file) {
      std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
      std::string upload_err;
      auto art = plan_manager_->upload_artifact("aavc2026_mission.plan", bytes, 0, &upload_err);
      if (art) {
        default_artifact_id = art->artifact_id;
        std::string wp_err;
        auto wp = plan_manager_->create_or_select_working_plan(
          default_artifact_id, "kmitl_airfield", "default_scenario", 0, &wp_err);
        if (wp) {
          default_wp_id = wp->get_working_plan_id();
          RCLCPP_INFO(get_logger(), "[RUNTIME] Loaded default plan artifact '%s' -> working plan '%s'",
            default_artifact_id.c_str(), default_wp_id.c_str());
        }
      }
    }
  }

  // Initialize Domain Context & Persistence
  context_ = std::make_shared<domain::MissionContext>("ctx_flight_runtime");
  context_->set_engineering_config(config_);

  // Pre-commit default simulation context for simulation bringup
  std::string err;
  context_->select_map_scenario("kmitl_airfield", "default_scenario", 0, &err);
  context_->select_target(domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026"), 1, &err);
  if (!default_artifact_id.empty()) {
    context_->select_plan_artifact(default_artifact_id, 2, &err);
  }
  if (!default_wp_id.empty()) {
    context_->select_working_plan(default_wp_id, 3, &err);
  }
  uint64_t next_rev = (!default_wp_id.empty()) ? 4 : 2;
  auto vreport = context_->validate_selection(next_rev);
  if (vreport.is_valid) {
    context_->commit(vreport.token, next_rev, &err);
  }

  persistence::StoragePaths paths;
  paths.state_directory = "/tmp/fsd_state";
  paths.evidence_directory = "/tmp/fsd_evidence";
  paths.plan_directory = "/tmp/fsd_plans";
  paths.backup_directory = "/tmp/fsd_backups";
  persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

  // Initialize Supervisor
  supervisor_ = std::make_shared<LifecycleSupervisor>();
  supervisor_->configure_all();
  supervisor_->activate_all();

  // Initialize PX4 Context & State Cache
  px4_context_ = std::make_unique<px4_ros2::Context>(*this);
  state_cache_ = std::make_shared<adapters::Px4StateCache>(*px4_context_);

  // Initialize Coordinator
  coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);
  coordinator_->set_plan_manager(plan_manager_);

  // Start periodic evaluation timer at 10Hz
  periodic_timer_ = this->create_wall_timer(
    100ms, std::bind(&FlightRuntimeNode::trigger_evaluation_cycle, this));
}

void FlightRuntimeNode::trigger_evaluation_cycle()
{
  check_and_register_mode();

  if (mode_ && mode_->isActive() && state_cache_ && state_cache_->is_armed()) {
    if (coordinator_ && coordinator_->get_current_strategy() == flight::StrategyType::WAITING_FOR_MODE) {
      auto snapshot = state_cache_->capture_snapshot();
      if (snapshot.is_landed) {
        RCLCPP_INFO(get_logger(), "[RUNTIME] Mode active on ground. Transitioning to TAKEOFF...");
        coordinator_->request_transition(flight::StrategyType::TAKEOFF);
      } else {
        RCLCPP_INFO(get_logger(), "[RUNTIME] Mode active airborne. Transitioning to TRANSIT_IN...");
        coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
      }
    }
  }

  publish_status_cycle();
}

void FlightRuntimeNode::check_and_register_mode()
{
  if (mode_registered_) {
    return;
  }

  // Evaluate all prerequisites before mode registration (Property 22)
  bool config_ok = (config_ && config_->validate().is_valid);
  auto rec_status = persistence_->get_recovery_status();
  bool recovery_ok = (rec_status.state == full_self_driving::msg::RecoveryStatus::STATE_CLEAR ||
                      rec_status.state == full_self_driving::msg::RecoveryStatus::STATE_RESOLVED);
  bool transport_ok = state_cache_->is_transport_healthy();

  std::vector<std::string> missing_gates;
  bool ready = supervisor_->evaluate_runtime_readiness(
    config_ok, true, recovery_ok, transport_ok, &missing_gates);

  if (ready && api_manifest_valid_) {
    RCLCPP_INFO(get_logger(), "[RUNTIME] All registration prerequisites met. Constructing and registering mode...");
    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*this, state_cache_);

    // Set arming check callback
    mode_->set_readiness_check_callback([this](std::vector<std::string> & failure_codes) -> bool {
      std::vector<std::string> missing;
      auto rec = persistence_->get_recovery_status();
      bool rec_ok = (rec.state == full_self_driving::msg::RecoveryStatus::STATE_CLEAR ||
                     rec.state == full_self_driving::msg::RecoveryStatus::STATE_RESOLVED);
      bool ok = context_->check_readiness(
        state_cache_->is_transport_healthy(),
        rec_ok,
        supervisor_->is_all_active(),
        &missing);
      failure_codes = missing;
      return ok;
    });

    mode_->set_activation_callback([this](bool is_active) {
      if (is_active && state_cache_->is_armed()) {
        if (coordinator_ && coordinator_->get_current_strategy() == flight::StrategyType::WAITING_FOR_MODE) {
          auto snapshot = state_cache_->capture_snapshot();
          if (snapshot.is_landed) {
            RCLCPP_INFO(get_logger(), "[RUNTIME] Mode activated on ground. Transitioning to TAKEOFF...");
            coordinator_->request_transition(flight::StrategyType::TAKEOFF);
          } else {
            RCLCPP_INFO(get_logger(), "[RUNTIME] Mode activated airborne. Transitioning to TRANSIT_IN...");
            coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
          }
        }
      }
    });

    mode_->set_strategy_completed_callback([this](flight::StrategyType completed_type) {
      if (completed_type == flight::StrategyType::TAKEOFF) {
        RCLCPP_INFO(get_logger(), "[RUNTIME] Takeoff completed. Transitioning to TRANSIT_IN...");
        if (coordinator_) {
          coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
        }
      } else if (completed_type == flight::StrategyType::TRANSIT_IN) {
        RCLCPP_INFO(get_logger(), "[RUNTIME] TransitIn completed. Transitioning to ACQUIRE_TARGET...");
        if (coordinator_) {
          coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET);
        }
      } else if (completed_type == flight::StrategyType::SEARCH) {
        RCLCPP_INFO(get_logger(), "[RUNTIME] Search completed. Holding over final search waypoint...");
      }
    });

    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*this, *mode_, state_cache_);
    if (config_) {
      executor_->set_takeoff_altitude(static_cast<float>(config_->routes.search_altitude_m));
    }
    executor_->set_takeover_callback([this](flight::FullSelfDrivingModeExecutor::DeactivateReason reason) {
      if (coordinator_) {
        coordinator_->handle_takeover(reason);
      }
    });

    if (executor_->doRegister()) {
      mode_registered_ = true;
      coordinator_->bind_executor(executor_, mode_);
      RCLCPP_INFO(get_logger(), "[RUNTIME] Successfully registered external mode 'Full Self-Driving' (mode_id=%d)",
        mode_->id());
    } else {
      RCLCPP_ERROR(get_logger(), "[RUNTIME] Mode registration with PX4 failed");
    }
  }
}

bool FlightRuntimeNode::is_ready_for_ownmode() const
{
  if (!mode_registered_ || !context_ || !persistence_ || !supervisor_ || !state_cache_) {
    return false;
  }
  return supervisor_->is_all_active() &&
         state_cache_->is_transport_healthy();
}

void FlightRuntimeNode::publish_status_cycle()
{
  state_sequence_++;
  auto now = this->get_clock()->now();

  // 1. FullSelfDrivingState
  full_self_driving::msg::FullSelfDrivingState state_msg;
  state_msg.header.stamp = now;
  state_msg.header.frame_id = "map";
  state_msg.config_state = static_cast<uint8_t>(context_->get_state());
  state_msg.flight_phase = coordinator_ ? coordinator_->get_flight_phase() : 1;
  state_msg.armed = state_cache_->is_armed();
  state_msg.locked = context_->is_locked();
  state_msg.ready_for_mode = is_ready_for_ownmode();
  state_msg.active_strategy = mode_ ? mode_->get_current_strategy_name() : "UNREGISTERED";
  state_msg.mission_id = context_->get_mission_id();
  state_msg.sortie_id = context_->get_sortie_id();
  state_msg.config_hash = context_->get_resolved_config_hash();
  state_msg.state_sequence = state_sequence_;
  state_msg.state_monotonic_ns = now.nanoseconds();
  state_pub_->publish(state_msg);

  // 2. ReadinessReport
  full_self_driving::msg::ReadinessReport readiness_msg;
  readiness_msg.ready = is_ready_for_ownmode();
  readiness_msg.readiness_revision = std::to_string(context_->get_committed_revision());
  readiness_msg.evaluated_at = now;
  readiness_msg.evaluated_monotonic_ns = now.nanoseconds();
  readiness_pub_->publish(readiness_msg);

  // 3. FlightSafetyStatus
  full_self_driving::msg::FlightSafetyStatus safety_msg;
  safety_msg.header.stamp = now;
  safety_msg.header.frame_id = "base_link";
  if (coordinator_ && coordinator_->is_emergency_stop_active()) {
    safety_msg.safety_status = full_self_driving::msg::FlightSafetyStatus::SAFETY_STATUS_EMERGENCY_STOP;
    safety_msg.emergency_stop_active = true;
  } else if (coordinator_ && coordinator_->is_takeover_active()) {
    safety_msg.safety_status = full_self_driving::msg::FlightSafetyStatus::SAFETY_STATUS_HOLD;
  } else {
    safety_msg.safety_status = full_self_driving::msg::FlightSafetyStatus::SAFETY_STATUS_OK;
  }
  safety_msg.safety_monotonic_ns = now.nanoseconds();
  safety_pub_->publish(safety_msg);

  // 4. VehicleTelemetry
  full_self_driving::msg::VehicleTelemetry telem_msg;
  telem_msg.header.stamp = now;
  telem_msg.header.frame_id = "base_link";
  auto snapshot = state_cache_->capture_snapshot();
  telem_msg.armed = snapshot.is_armed;
  telem_msg.airborne = !snapshot.is_landed;
  telem_msg.landed = snapshot.is_landed;
  telem_msg.battery_percentage = 100.0f;
  telem_msg.voltage_v = 16.8f;
  telem_msg.current_a = 0.0f;
  telem_msg.latitude_deg = snapshot.global_position.x();
  telem_msg.longitude_deg = snapshot.global_position.y();
  telem_msg.altitude_m = snapshot.global_position.z();
  telem_msg.heading_deg = snapshot.heading * 180.0f / 3.141592653589793f;
  telem_msg.ground_speed_m_s = std::sqrt(snapshot.local_velocity_ned.x() * snapshot.local_velocity_ned.x() +
                                         snapshot.local_velocity_ned.y() * snapshot.local_velocity_ned.y());
  telem_msg.vertical_speed_m_s = -snapshot.local_velocity_ned.z();
  telem_msg.telemetry_monotonic_ns = snapshot.monotonic_timestamp_ns;
  telemetry_pub_->publish(telem_msg);

  // 5. WorkingPlanStatus
  if (plan_manager_) {
    std::string wp_id = context_->get_selection().working_plan_id;
    std::optional<domain::WorkingPlan> active_wp;
    if (!wp_id.empty()) {
      active_wp = plan_manager_->get_working_plan(wp_id);
    }
    if (!active_wp) {
      active_wp = plan_manager_->get_active_working_plan("kmitl_airfield", "default_scenario");
    }
    if (active_wp) {
      full_self_driving::msg::WorkingPlanStatus wp_msg;
      wp_msg.header.stamp = now;
      wp_msg.working_plan_id = active_wp->get_working_plan_id();
      wp_msg.map_id = active_wp->get_map_id();
      wp_msg.scenario_id = active_wp->get_scenario_id();
      wp_msg.source_artifact_sha256 = active_wp->get_source_artifact_sha256();
      wp_msg.canonical_route_sha256 = active_wp->get_canonical_route_sha256();
      wp_msg.generation = active_wp->get_generation();
      wp_msg.state = static_cast<uint8_t>(active_wp->get_state());
      wp_msg.durability_state = static_cast<uint8_t>(active_wp->get_durability_state());
      wp_msg.update_reason = active_wp->get_checkpoint().checkpoint_reason;
      wp_msg.updated_at = now;

      const auto & cp = active_wp->get_checkpoint();
      wp_msg.checkpoint.working_plan_id = cp.working_plan_id;
      wp_msg.checkpoint.generation = cp.generation;
      wp_msg.checkpoint.next_source_index = cp.next_source_index;
      wp_msg.checkpoint.has_checkpoint_position = cp.has_checkpoint_position;
      wp_msg.checkpoint.checkpoint_latitude_deg = cp.checkpoint_latitude_deg;
      wp_msg.checkpoint.checkpoint_longitude_deg = cp.checkpoint_longitude_deg;
      wp_msg.checkpoint.checkpoint_altitude_m = cp.checkpoint_altitude_m;
      wp_msg.checkpoint.completed_waypoints = cp.completed_waypoints;
      wp_msg.checkpoint.total_waypoints = cp.total_waypoints;
      wp_msg.checkpoint.progress_percent = cp.progress_percent;
      wp_msg.checkpoint.checkpoint_reason = cp.checkpoint_reason;
      wp_msg.checkpoint.checkpoint_sequence = cp.checkpoint_sequence;
      wp_msg.checkpoint.updated_at = now;
      wp_msg.checkpoint.updated_monotonic_ns = now.nanoseconds();

      working_plan_status_pub_->publish(wp_msg);
    }
  }
}

}  // namespace full_self_driving::runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<full_self_driving::runtime::FlightRuntimeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
