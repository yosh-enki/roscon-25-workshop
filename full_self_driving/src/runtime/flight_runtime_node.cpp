#include "runtime/flight_runtime_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace full_self_driving::runtime
{

FlightRuntimeNode::FlightRuntimeNode(const rclcpp::NodeOptions & options)
: Node("fsd_flight_runtime", options)
{
  this->declare_parameter<std::string>("engineering_config", "");
  this->declare_parameter<std::string>("pinned_api_manifest", "");
  this->declare_parameter<bool>("simulation", true);
  this->declare_parameter<std::string>("world", "kmitl_airfield");

  config_path_ = this->get_parameter("engineering_config").as_string();
  manifest_path_ = this->get_parameter("pinned_api_manifest").as_string();

  if (manifest_path_.empty()) {
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("full_self_driving");
      manifest_path_ = pkg_share + "/config/pinned_api_manifest.yaml";
    } catch (const std::exception &) {
      manifest_path_ = "config/pinned_api_manifest.yaml";
    }
  }

  initialize_components();
}

void FlightRuntimeNode::initialize_components()
{
  // Publishers
  state_pub_ = this->create_publisher<full_self_driving::msg::FullSelfDrivingState>(
    "/full_self_driving/state", rclcpp::QoS(10));
  readiness_pub_ = this->create_publisher<full_self_driving::msg::ReadinessReport>(
    "/full_self_driving/readiness", rclcpp::QoS(10));
  safety_pub_ = this->create_publisher<full_self_driving::msg::FlightSafetyStatus>(
    "/full_self_driving/safety_status", rclcpp::QoS(10));
  telemetry_pub_ = this->create_publisher<full_self_driving::msg::VehicleTelemetry>(
    "/full_self_driving/vehicle_telemetry", rclcpp::QoS(10));

  // Subscriptions & Services
  target_lock_sub_ = this->create_subscription<full_self_driving::msg::LiveTargetLock>(
    "/full_self_driving/perception/live_target_lock", rclcpp::QoS(10),
    [this](const full_self_driving::msg::LiveTargetLock::SharedPtr msg) {
      if (coordinator_) {
        domain::LiveTargetLock lock = domain::LiveTargetLock::from_msg(*msg);
        coordinator_->handle_target_lock_update(lock);
      }
    });

  emergency_stop_srv_ = this->create_service<full_self_driving::srv::EmergencyStop>(
    "/full_self_driving/emergency_stop",
    [this](const std::shared_ptr<full_self_driving::srv::EmergencyStop::Request> req,
           std::shared_ptr<full_self_driving::srv::EmergencyStop::Response> res) {
      RCLCPP_WARN(get_logger(), "[RUNTIME] EmergencyStop requested: %s", req->reason.c_str());
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

  // Initialize Domain Context & Persistence
  context_ = std::make_shared<domain::MissionContext>("ctx_flight_runtime");
  context_->set_engineering_config(config_);

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

  // Start periodic evaluation timer at 10Hz
  periodic_timer_ = this->create_wall_timer(
    100ms, std::bind(&FlightRuntimeNode::trigger_evaluation_cycle, this));
}

void FlightRuntimeNode::trigger_evaluation_cycle()
{
  check_and_register_mode();
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

    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*this, *mode_);
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
