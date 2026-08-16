#include "search/SearchBridge.hpp"
#include "search/PlanParser.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

constexpr char kVehicleStatusTopic[] = "/fmu/out/vehicle_status_v1";
constexpr char kVehicleLandDetectedTopic[] = "/fmu/out/vehicle_land_detected";
constexpr std::size_t kMaximumQueuedCommands = 64U;
constexpr std::size_t kMaximumProcessedRequestIds = 256U;
constexpr std::uint64_t kLikelyPx4RebootTimestampUs = 10'000'000U;

const char * environmentValue(const char * name)
{
	const char * value = std::getenv(name);
	return value == nullptr ? "" : value;
}

search::Json jsonArray(const std::vector<std::string> & values)
{
	std::vector<search::Json> items;
	items.reserve(values.size());
	for (const auto & value : values) {
		items.push_back(search::Json::makeString(value));
	}
	return search::Json::makeArray(std::move(items));
}

}  // namespace

SearchBridge::SearchBridge(const rclcpp::NodeOptions & options)
: Node("search_bridge", options)
{
	loadParameters();
	resolvePlanDirectories();
	configureTopics();

	_planner = std::make_unique<search::SearchPlanner>(
		_manual_plan_directory,
		_working_plan_directory,
		_default_manual_plan,
		_reset_working_plan);

	_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
		kVehicleStatusTopic,
		rclcpp::QoS(1).best_effort(),
		std::bind(&SearchBridge::vehicleStatusCallback, this, std::placeholders::_1));
	_vehicle_land_detected_sub = create_subscription<px4_msgs::msg::VehicleLandDetected>(
		kVehicleLandDetectedTopic,
		rclcpp::QoS(1).best_effort(),
		std::bind(&SearchBridge::vehicleLandDetectedCallback, this, std::placeholders::_1));

	const int timer_period_ms = std::min(
		_status_publish_period_ms, _planner_publish_period_ms);
	_timer = create_wall_timer(
		std::chrono::milliseconds(timer_period_ms),
		std::bind(&SearchBridge::processTimer, this));

	configureMqtt();
	RCLCPP_INFO(
		get_logger(),
		"search_bridge initialized. PX4 status='%s', MQTT status='%s', planner plans='%s'",
		kVehicleStatusTopic, _status_topic.c_str(), _plans_topic.c_str());
}

SearchBridge::~SearchBridge()
{
	cleanupMqtt();
}

void SearchBridge::loadParameters()
{
	declare_parameter<std::string>("manual_plan_directory", "");
	declare_parameter<std::string>("working_plan_directory", "");
	declare_parameter<std::string>("default_manual_plan", "aavc2026_mission.plan");
	declare_parameter<bool>("reset_working_plan", false);

	declare_parameter<std::string>("mqtt_host", "");
	declare_parameter<int>("mqtt_port", 8883);
	declare_parameter<std::string>("mqtt_username", "");
	declare_parameter<std::string>("mqtt_password", "");
	declare_parameter<std::string>("mqtt_client_id", "search_bridge");
	declare_parameter<std::string>("mqtt_topic_prefix", "search");
	declare_parameter<std::string>("mqtt_tls_ca_file", "");
	declare_parameter<double>("px4_status_timeout_s", 2.0);
	declare_parameter<int>("status_publish_period_ms", 500);
	declare_parameter<int>("planner_publish_period_ms", 2000);

	get_parameter("manual_plan_directory", _manual_plan_directory);
	get_parameter("working_plan_directory", _working_plan_directory);
	get_parameter("default_manual_plan", _default_manual_plan);
	get_parameter("reset_working_plan", _reset_working_plan);
	get_parameter("mqtt_host", _mqtt_host);
	get_parameter("mqtt_port", _mqtt_port);
	get_parameter("mqtt_username", _mqtt_username);
	get_parameter("mqtt_password", _mqtt_password);
	get_parameter("mqtt_client_id", _mqtt_client_id);
	get_parameter("mqtt_topic_prefix", _mqtt_topic_prefix);
	get_parameter("mqtt_tls_ca_file", _mqtt_tls_ca_file);
	get_parameter("px4_status_timeout_s", _px4_status_timeout_s);
	get_parameter("status_publish_period_ms", _status_publish_period_ms);
	get_parameter("planner_publish_period_ms", _planner_publish_period_ms);

	// Credentials are intentionally read from the environment when the ROS
	// parameters are left empty. They are never printed or put in a payload.
	if (_mqtt_username.empty()) {
		_mqtt_username = environmentValue("HIVEMQ_MQTT_USERNAME");
	}
	if (_mqtt_password.empty()) {
		_mqtt_password = environmentValue("HIVEMQ_MQTT_PASSWORD");
	}

	if (_mqtt_port < 1 || _mqtt_port > 65535) {
		throw std::invalid_argument("search_bridge: mqtt_port must be between 1 and 65535");
	}
	if (!std::isfinite(_px4_status_timeout_s) || _px4_status_timeout_s <= 0.0) {
		throw std::invalid_argument(
			"search_bridge: px4_status_timeout_s must be finite and positive");
	}
	if (_status_publish_period_ms < 100 || _planner_publish_period_ms < 100) {
		throw std::invalid_argument(
			"search_bridge: publish periods must be at least 100 milliseconds");
	}
	if (_mqtt_client_id.empty()) {
		throw std::invalid_argument("search_bridge: mqtt_client_id must not be empty");
	}
	if (_mqtt_username.empty() != _mqtt_password.empty()) {
		throw std::invalid_argument(
			"search_bridge: provide both MQTT credentials or leave both empty");
	}
	if (!_mqtt_host.empty() && (_mqtt_username.empty() || _mqtt_password.empty())) {
		throw std::invalid_argument(
			"search_bridge: MQTT credentials are required when mqtt_host is configured");
	}
}

void SearchBridge::resolvePlanDirectories()
{
	std::string package_share;
	if (_manual_plan_directory.empty() || _working_plan_directory.empty()) {
		try {
			package_share = ament_index_cpp::get_package_share_directory("search");
		} catch (const std::exception & error) {
			RCLCPP_WARN(
				get_logger(),
				"Could not resolve search package share directory (%s); "
				"using paths relative to the current working directory",
				error.what());
		}
	}

	if (_manual_plan_directory.empty()) {
		_manual_plan_directory = package_share.empty()
			? "plans/manual"
			: package_share + "/plans/manual";
	}
	if (_working_plan_directory.empty()) {
		_working_plan_directory = package_share.empty()
			? "plans/working"
			: package_share + "/plans/working";
	}
}

void SearchBridge::configureTopics()
{
	while (!_mqtt_topic_prefix.empty() && _mqtt_topic_prefix.front() == '/') {
		_mqtt_topic_prefix.erase(0U, 1U);
	}
	while (!_mqtt_topic_prefix.empty() && _mqtt_topic_prefix.back() == '/') {
		_mqtt_topic_prefix.pop_back();
	}
	if (_mqtt_topic_prefix.empty() || _mqtt_topic_prefix.find('#') != std::string::npos ||
		_mqtt_topic_prefix.find('+') != std::string::npos) {
		throw std::invalid_argument(
			"search_bridge: mqtt_topic_prefix must be a non-empty concrete topic prefix");
	}

	_status_topic = _mqtt_topic_prefix + "/px4/status";
	_plans_topic = _mqtt_topic_prefix + "/planner/plans";
	_command_topic = _mqtt_topic_prefix + "/planner/command";
	_response_topic = _mqtt_topic_prefix + "/planner/response";
	_planner_status_topic = _mqtt_topic_prefix + "/planner/status";
}

void SearchBridge::configureMqtt()
{
	if (_mqtt_host.empty()) {
		RCLCPP_ERROR(
			get_logger(),
			"mqtt_host is empty; search_bridge will run ROS subscriptions but will not connect to MQTT");
		return;
	}

	int result = mosquitto_lib_init();
	if (result != MOSQ_ERR_SUCCESS) {
		throw std::runtime_error(
			std::string("search_bridge: mosquitto_lib_init failed: ") + mosquitto_strerror(result));
	}
	_mosquitto_initialized = true;

	_mosquitto = mosquitto_new(_mqtt_client_id.c_str(), true, this);
	if (_mosquitto == nullptr) {
		cleanupMqtt();
		throw std::runtime_error("search_bridge: cannot create the MQTT client");
	}

	mosquitto_connect_callback_set(_mosquitto, &SearchBridge::mqttConnectCallback);
	mosquitto_disconnect_callback_set(_mosquitto, &SearchBridge::mqttDisconnectCallback);
	mosquitto_message_callback_set(_mosquitto, &SearchBridge::mqttMessageCallback);
	(void)mosquitto_reconnect_delay_set(_mosquitto, 2U, 30U, true);

	// The planner-status topic is the Node-RED safety authority. Its retained
	// Last Will makes an unexpected bridge loss fail closed for plan controls.
	const std::string offline_planner_status = offlinePlannerStatusPayload();
	result = mosquitto_will_set(
		_mosquitto,
		_planner_status_topic.c_str(),
		static_cast<int>(offline_planner_status.size()),
		offline_planner_status.data(),
		1,
		true);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string message =
			std::string("search_bridge: MQTT offline status setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(message);
	}

	const char * ca_file = _mqtt_tls_ca_file.empty() ? nullptr : _mqtt_tls_ca_file.c_str();
	const char * ca_path = _mqtt_tls_ca_file.empty() ? "/etc/ssl/certs" : nullptr;
	result = mosquitto_tls_set(_mosquitto, ca_file, ca_path, nullptr, nullptr, nullptr);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string message =
			std::string("search_bridge: MQTT TLS setup failed: ") + mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(message);
	}
	result = mosquitto_tls_insecure_set(_mosquitto, false);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string message =
			std::string("search_bridge: MQTT certificate verification setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(message);
	}

	if (!_mqtt_username.empty()) {
		result = mosquitto_username_pw_set(
			_mosquitto, _mqtt_username.c_str(), _mqtt_password.c_str());
		if (result != MOSQ_ERR_SUCCESS) {
			const std::string message =
				std::string("search_bridge: MQTT credential setup failed: ") +
				mosquitto_strerror(result);
			cleanupMqtt();
			throw std::runtime_error(message);
		}
	}

	result = mosquitto_connect_async(_mosquitto, _mqtt_host.c_str(), _mqtt_port, 60);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string message =
			std::string("search_bridge: MQTT connection setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(message);
	}

	result = mosquitto_loop_start(_mosquitto);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string message =
			std::string("search_bridge: MQTT network loop failed to start: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(message);
	}
	_mqtt_loop_started = true;
	RCLCPP_INFO(get_logger(), "Connecting to MQTT broker '%s:%d' with TLS", _mqtt_host.c_str(), _mqtt_port);
}

void SearchBridge::cleanupMqtt()
{
	std::lock_guard<std::mutex> lock(_mqtt_mutex);
	if (_mosquitto != nullptr && _mqtt_connected.load()) {
		const std::string offline_status = offlineStatusPayload();
		const std::string offline_planner_status = offlinePlannerStatusPayload();
		(void)mosquitto_publish(
			_mosquitto,
			nullptr,
			_status_topic.c_str(),
			static_cast<int>(offline_status.size()),
			offline_status.data(),
			1,
			true);
		(void)mosquitto_publish(
			_mosquitto,
			nullptr,
			_planner_status_topic.c_str(),
			static_cast<int>(offline_planner_status.size()),
			offline_planner_status.data(),
			1,
			true);
	}
	_mqtt_connected.store(false);
	if (_mosquitto != nullptr) {
		if (_mqtt_loop_started) {
			(void)mosquitto_disconnect(_mosquitto);
			(void)mosquitto_loop_stop(_mosquitto, true);
			_mqtt_loop_started = false;
		}
		mosquitto_destroy(_mosquitto);
		_mosquitto = nullptr;
	}
	if (_mosquitto_initialized) {
		mosquitto_lib_cleanup();
		_mosquitto_initialized = false;
	}
}

void SearchBridge::vehicleStatusCallback(
	const px4_msgs::msg::VehicleStatus::SharedPtr message)
{
	if (message == nullptr) {
		return;
	}
	{
		std::lock_guard<std::mutex> lock(_status_mutex);
		const std::uint64_t timestamp = message->timestamp;
		const bool looks_like_reboot = timestamp != 0U &&
			timestamp < kLikelyPx4RebootTimestampUs;
		if (timestamp != 0U && _vehicle_status_timestamp_valid &&
			timestamp <= _last_vehicle_status_timestamp && !looks_like_reboot) {
			// Do not let delayed/out-of-order DDS samples refresh the safety
			// watchdog or replace a newer PX4 status.
			return;
		}
		_vehicle_status = *message;
		_vehicle_status_received_at = SteadyClock::now();
		_vehicle_status_received_wall = std::chrono::system_clock::now();
		_vehicle_status_timestamp_valid = timestamp != 0U;
		if (_vehicle_status_timestamp_valid) {
			_last_vehicle_status_timestamp = timestamp;
		}
	}

	const auto now = SteadyClock::now();
	if (_last_status_publish.time_since_epoch().count() == 0 ||
		std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_status_publish).count() >=
			_status_publish_period_ms) {
		publishStatus();
	}
}

void SearchBridge::vehicleLandDetectedCallback(
	const px4_msgs::msg::VehicleLandDetected::SharedPtr message)
{
	if (message == nullptr) {
		return;
	}
	std::lock_guard<std::mutex> lock(_status_mutex);
	_landed = message->landed;
}

SearchBridge::SafetySnapshot SearchBridge::safetySnapshot() const
{
	SafetySnapshot result;
	std::lock_guard<std::mutex> lock(_status_mutex);
	if (!_vehicle_status.has_value()) {
		return result;
	}

	result.known = true;
	result.arming_state = _vehicle_status->arming_state;
	const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
		SteadyClock::now() - _vehicle_status_received_at);
	result.age_ms = std::max(0.0, static_cast<double>(age.count()));
	result.fresh = _vehicle_status_timestamp_valid &&
		result.age_ms <= (_px4_status_timeout_s * 1000.0);
	result.disarmed = result.fresh &&
		_vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
	return result;
}

std::string SearchBridge::safetyFailureReason(const SafetySnapshot & safety) const
{
	if (!safety.known) {
		return "PX4 VehicleStatus is unknown; planner management is disabled";
	}
	if (!safety.fresh) {
		return "PX4 VehicleStatus is stale; planner management is disabled";
	}
	if (!safety.disarmed) {
		return "Planner management is allowed only while PX4 is DISARMED";
	}
	return {};
}

std::string SearchBridge::armingStateName(std::uint8_t arming_state)
{
	if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
		return "DISARMED";
	}
	if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
		return "ARMED";
	}
	return "UNKNOWN";
}

std::int64_t SearchBridge::unixTimeMilliseconds()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string SearchBridge::offlineStatusPayload() const
{
	search::Json root = search::Json::makeObject();
	root.set("source", search::Json::makeString("px4"));
	root.set("topic", search::Json::makeString(kVehicleStatusTopic));
	root.set("sequence", search::Json::makeNumber(static_cast<double>(_status_sequence)));
	root.set("status_state", search::Json::makeString("OFFLINE"));
	root.set("mqtt_connected", search::Json::makeBoolean(false));
	root.set("px4_timestamp_us", search::Json::makeNull());
	root.set("px4_timestamp_valid", search::Json::makeBoolean(false));
	root.set("received_at_unix_ms", search::Json::makeNull());
	root.set("status_age_ms", search::Json::makeNull());
	root.set("arming_state", search::Json::makeNull());
	root.set("arming_state_name", search::Json::makeString("UNKNOWN"));
	root.set("armed", search::Json::makeNull());
	root.set("nav_state", search::Json::makeNull());
	root.set("failsafe", search::Json::makeNull());
	root.set("failsafe_and_user_took_over", search::Json::makeNull());
	root.set("pre_flight_checks_pass", search::Json::makeNull());
	root.set("gcs_connection_lost", search::Json::makeNull());
	root.set("is_vtol", search::Json::makeNull());
	root.set("system_id", search::Json::makeNull());
	root.set("component_id", search::Json::makeNull());
	root.set("armed_time", search::Json::makeNull());
	root.set("takeoff_time", search::Json::makeNull());
	root.set("landed", search::Json::makeNull());
	root.set("plan_management_allowed", search::Json::makeBoolean(false));
	root.set(
		"plan_management_message",
		search::Json::makeString("search_bridge is offline; planner management is disabled"));
	return root.dump();
}

std::string SearchBridge::offlinePlannerStatusPayload() const
{
	search::Json root = search::Json::makeObject();
	root.set("source", search::Json::makeString("search_bridge"));
	root.set("topic", search::Json::makeString(_planner_status_topic));
	root.set("status_state", search::Json::makeString("OFFLINE"));
	root.set("status_age_ms", search::Json::makeNull());
	root.set("arming_state", search::Json::makeNull());
	root.set("arming_state_name", search::Json::makeString("UNKNOWN"));
	root.set("plan_management_allowed", search::Json::makeBoolean(false));
	root.set(
		"active_working_plan",
		search::Json::makeString(_planner->activeWorkingPlanName()));
	root.set("last_error", search::Json::makeString(_last_planner_error));
	root.set(
		"message",
		search::Json::makeString("search_bridge is offline; planner management is disabled"));
	return root.dump();
}

std::string SearchBridge::statusPayload()
{
	const auto now = SteadyClock::now();
	search::Json root = search::Json::makeObject();
	root.set("source", search::Json::makeString("px4"));
	root.set("topic", search::Json::makeString(kVehicleStatusTopic));
	root.set("sequence", search::Json::makeNumber(static_cast<double>(++_status_sequence)));
	root.set("mqtt_connected", search::Json::makeBoolean(_mqtt_connected.load()));

	std::lock_guard<std::mutex> lock(_status_mutex);
	if (!_vehicle_status.has_value()) {
		root.set("status_state", search::Json::makeString("UNKNOWN"));
		root.set("px4_timestamp_us", search::Json::makeNull());
		root.set("received_at_unix_ms", search::Json::makeNull());
		root.set("status_age_ms", search::Json::makeNull());
		root.set("px4_timestamp_valid", search::Json::makeBoolean(false));
		root.set("arming_state", search::Json::makeNull());
		root.set("arming_state_name", search::Json::makeString("UNKNOWN"));
		root.set("armed", search::Json::makeNull());
		root.set("nav_state", search::Json::makeNull());
		root.set("failsafe", search::Json::makeNull());
		root.set("failsafe_and_user_took_over", search::Json::makeNull());
		root.set("pre_flight_checks_pass", search::Json::makeNull());
		root.set("gcs_connection_lost", search::Json::makeNull());
		root.set("is_vtol", search::Json::makeNull());
		root.set("system_id", search::Json::makeNull());
		root.set("component_id", search::Json::makeNull());
		root.set("armed_time", search::Json::makeNull());
		root.set("takeoff_time", search::Json::makeNull());
		root.set("landed", _landed.has_value()
			? search::Json::makeBoolean(*_landed) : search::Json::makeNull());
		root.set("plan_management_allowed", search::Json::makeBoolean(false));
		root.set(
			"plan_management_message",
			search::Json::makeString("PX4 VehicleStatus is unknown; planner management is disabled"));
		return root.dump();
	}

	const auto & status = *_vehicle_status;
	const double age_ms = std::max(
		0.0,
		static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
			now - _vehicle_status_received_at).count()));
	const bool fresh = _vehicle_status_timestamp_valid &&
		age_ms <= (_px4_status_timeout_s * 1000.0);
	const bool disarmed = fresh &&
		status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;
	root.set("status_state", search::Json::makeString(fresh ? "FRESH" : "STALE"));
	root.set("px4_timestamp_valid", search::Json::makeBoolean(_vehicle_status_timestamp_valid));
	root.set("px4_timestamp_us", search::Json::makeNumber(static_cast<double>(status.timestamp)));
	root.set(
		"received_at_unix_ms",
		search::Json::makeNumber(static_cast<double>(
			std::chrono::duration_cast<std::chrono::milliseconds>(
				_vehicle_status_received_wall.time_since_epoch()).count())));
	root.set("status_age_ms", search::Json::makeNumber(age_ms));
	root.set("arming_state", search::Json::makeNumber(static_cast<double>(status.arming_state)));
	root.set("arming_state_name", search::Json::makeString(armingStateName(status.arming_state)));
	root.set(
		"armed",
		search::Json::makeBoolean(
			status.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED));
	root.set("nav_state", search::Json::makeNumber(static_cast<double>(status.nav_state)));
	root.set("failsafe", search::Json::makeBoolean(status.failsafe));
	root.set(
		"failsafe_and_user_took_over",
		search::Json::makeBoolean(status.failsafe_and_user_took_over));
	root.set(
		"pre_flight_checks_pass",
		search::Json::makeBoolean(status.pre_flight_checks_pass));
	root.set("gcs_connection_lost", search::Json::makeBoolean(status.gcs_connection_lost));
	root.set("is_vtol", search::Json::makeBoolean(status.is_vtol));
	root.set("system_id", search::Json::makeNumber(static_cast<double>(status.system_id)));
	root.set("component_id", search::Json::makeNumber(static_cast<double>(status.component_id)));
	root.set("armed_time", search::Json::makeNumber(static_cast<double>(status.armed_time)));
	root.set("takeoff_time", search::Json::makeNumber(static_cast<double>(status.takeoff_time)));
	root.set("landed", _landed.has_value()
		? search::Json::makeBoolean(*_landed) : search::Json::makeNull());
	root.set("plan_management_allowed", search::Json::makeBoolean(disarmed));
	root.set(
		"plan_management_message",
		search::Json::makeString(
			disarmed ? "Planner management is allowed while PX4 is DISARMED" :
				"Planner management is disabled until PX4 is DISARMED and VehicleStatus is fresh"));
	return root.dump();
}

std::string SearchBridge::plansPayload()
{
	search::Json root = search::Json::makeObject();
	root.set("source", search::Json::makeString("search_bridge"));
	root.set("published_at_unix_ms", search::Json::makeNumber(
		static_cast<double>(unixTimeMilliseconds())));
	root.set("default_manual_plan", search::Json::makeString(_default_manual_plan));

	try {
		_planner->refreshActiveWorkingPlan();
		const std::vector<std::string> plans = _planner->listManualPlanNames();
		root.set("plans", jsonArray(plans));
		root.set(
			"active_working_plan",
			search::Json::makeString(_planner->activeWorkingPlanName()));
		root.set("available", search::Json::makeBoolean(true));
		root.set("message", search::Json::makeString("Manual plans are available"));
		_last_planner_error.clear();
	} catch (const std::exception & error) {
		root.set("plans", search::Json::makeArray(std::vector<search::Json>{}));
		root.set(
			"active_working_plan",
			search::Json::makeString(_planner->activeWorkingPlanName()));
		root.set("available", search::Json::makeBoolean(false));
		root.set("message", search::Json::makeString(error.what()));
		_last_planner_error = error.what();
	}
	return root.dump();
}

std::string SearchBridge::plannerStatusPayload() const
{
	const SafetySnapshot safety = safetySnapshot();
	search::Json root = search::Json::makeObject();
	root.set("source", search::Json::makeString("search_bridge"));
	root.set("topic", search::Json::makeString(_planner_status_topic));
	root.set("status_state", search::Json::makeString(
		safety.known ? (safety.fresh ? (safety.disarmed ? "DISARMED" : "NOT_DISARMED") : "STALE") :
			"UNKNOWN"));
	root.set("status_age_ms", safety.known
		? search::Json::makeNumber(safety.age_ms) : search::Json::makeNull());
	root.set("arming_state", safety.known
		? search::Json::makeNumber(static_cast<double>(safety.arming_state))
		: search::Json::makeNull());
	root.set("arming_state_name", search::Json::makeString(
		safety.known ? armingStateName(safety.arming_state) : "UNKNOWN"));
	root.set("plan_management_allowed", search::Json::makeBoolean(safety.disarmed));
	root.set(
		"active_working_plan",
		search::Json::makeString(_planner->activeWorkingPlanName()));
	root.set("last_error", search::Json::makeString(_last_planner_error));
	root.set(
		"message",
		search::Json::makeString(
			safety.disarmed ? "Planner management is allowed while PX4 is DISARMED" :
				safetyFailureReason(safety)));
	return root.dump();
}

void SearchBridge::publishStatus()
{
	if (!_mqtt_connected.load()) {
		return;
	}
	publishJson(_status_topic, statusPayload(), 1, true);
	_last_status_publish = SteadyClock::now();
}

void SearchBridge::publishPlans()
{
	if (!_mqtt_connected.load()) {
		return;
	}
	const SafetySnapshot safety = safetySnapshot();
	if (!safety.disarmed) {
		// Do not touch the filesystem while armed or while PX4 status is unknown.
		// The last retained valid plans payload remains available to Node-RED.
		return;
	}
	publishJson(_plans_topic, plansPayload(), 1, true);
}

void SearchBridge::publishPlannerStatus()
{
	if (!_mqtt_connected.load()) {
		return;
	}
	publishJson(_planner_status_topic, plannerStatusPayload(), 1, true);
	_last_planner_publish = SteadyClock::now();
}

void SearchBridge::publishResponse(
	const std::string & request_id,
	bool success,
	const std::string & message,
	const std::optional<std::string> & active_working_plan)
{
	search::Json root = search::Json::makeObject();
	root.set("request_id", search::Json::makeString(request_id));
	root.set("success", search::Json::makeBoolean(success));
	root.set("message", search::Json::makeString(message));
	root.set("responded_at_unix_ms", search::Json::makeNumber(
		static_cast<double>(unixTimeMilliseconds())));
	if (active_working_plan.has_value()) {
		root.set("active_working_plan", search::Json::makeString(*active_working_plan));
	}
	const std::string response_payload = root.dump();
	cacheResponse(request_id, response_payload);
	publishJson(_response_topic, response_payload, 1, false);
}

void SearchBridge::publishJson(
	const std::string & topic,
	const std::string & payload,
	int qos,
	bool retain)
{
	std::lock_guard<std::mutex> lock(_mqtt_mutex);
	if (_mosquitto == nullptr || !_mqtt_connected.load()) {
		return;
	}
	if (payload.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
		RCLCPP_ERROR(get_logger(), "MQTT payload is too large for libmosquitto");
		return;
	}
	const int result = mosquitto_publish(
		_mosquitto,
		nullptr,
		topic.c_str(),
		static_cast<int>(payload.size()),
		payload.data(),
		qos,
		retain);
	if (result != MOSQ_ERR_SUCCESS) {
		RCLCPP_WARN(
			get_logger(), "MQTT publish to '%s' failed: %s", topic.c_str(), mosquitto_strerror(result));
	}
}

void SearchBridge::processTimer()
{
	if (!_mqtt_connected.load()) {
		return;
	}

	if (_publish_initial_state.exchange(false)) {
		publishStatus();
		publishPlans();
		publishPlannerStatus();
	}

	processQueuedCommands();

	const auto now = SteadyClock::now();
	if (_last_status_publish.time_since_epoch().count() == 0 ||
		std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_status_publish).count() >=
			_status_publish_period_ms) {
		publishStatus();
	}
	if (_last_planner_publish.time_since_epoch().count() == 0 ||
		std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_planner_publish).count() >=
			_planner_publish_period_ms) {
		publishPlans();
		publishPlannerStatus();
	}
}

void SearchBridge::processQueuedCommands()
{
	std::deque<std::string> commands;
	{
		std::lock_guard<std::mutex> lock(_command_mutex);
		commands.swap(_command_queue);
	}
	for (const auto & command : commands) {
		handleCommand(command);
	}
}

void SearchBridge::cacheResponse(const std::string & request_id, const std::string & payload)
{
	if (request_id.empty()) {
		return;
	}
	if (_processed_responses.find(request_id) == _processed_responses.end()) {
		_processed_request_order.push_back(request_id);
	}
	_processed_responses[request_id] = payload;
	while (_processed_request_order.size() > kMaximumProcessedRequestIds) {
		_processed_responses.erase(_processed_request_order.front());
		_processed_request_order.pop_front();
	}
}

bool SearchBridge::isRequestIdProcessed(const std::string & request_id) const
{
	return _processed_responses.find(request_id) != _processed_responses.end();
}

void SearchBridge::clearQueuedCommands()
{
	std::lock_guard<std::mutex> lock(_command_mutex);
	_command_queue.clear();
}

void SearchBridge::handleCommand(const std::string & payload)
{
	std::string request_id;
	std::string command;
	std::string plan_name;
	try {
		const search::Json root = search::Json::parse(payload);
		if (!root.isObject()) {
			throw std::runtime_error("command payload must be a JSON object");
		}
		if (!root.has("request_id") || root.at("request_id").type() != search::Json::Type::String ||
			root.at("request_id").asString().empty()) {
			throw std::runtime_error("request_id must be a non-empty string");
		}
		request_id = root.at("request_id").asString();
		if (!root.has("command") || root.at("command").type() != search::Json::Type::String) {
			throw std::runtime_error("command must be a string");
		}
		command = root.at("command").asString();
		if (root.has("plan_name")) {
			if (root.at("plan_name").type() != search::Json::Type::String) {
				throw std::runtime_error("plan_name must be a string");
			}
			plan_name = root.at("plan_name").asString();
		}
	} catch (const std::exception & error) {
		publishResponse(request_id, false, error.what());
		return;
	}

	if (isRequestIdProcessed(request_id)) {
		const auto response = _processed_responses.find(request_id);
		if (response != _processed_responses.end()) {
			publishJson(_response_topic, response->second, 1, false);
		}
		return;
	}

	if (command != "list_manual_plans" && command != "reset_working_plan") {
		publishResponse(request_id, false, "Unsupported planner command");
		return;
	}

	const SafetySnapshot safety = safetySnapshot();
	if (!safety.disarmed) {
		publishResponse(request_id, false, safetyFailureReason(safety));
		return;
	}

	if (command == "list_manual_plans") {
		publishPlans();
		if (_last_planner_error.empty()) {
			publishResponse(request_id, true, "Manual plans published");
		} else {
			publishResponse(request_id, false, _last_planner_error);
		}
		return;
	}

	if (plan_name.empty()) {
		publishResponse(request_id, false, "plan_name is required for reset_working_plan");
		return;
	}

	try {
		const std::string active_working_plan = _planner->resetWorkingPlan(plan_name);
		_last_planner_error.clear();
		publishPlans();
		publishPlannerStatus();
		publishResponse(
			request_id,
			true,
			"Working plan reset from " + plan_name,
			active_working_plan);
	} catch (const std::exception & error) {
		_last_planner_error = error.what();
		publishPlannerStatus();
		publishResponse(request_id, false, error.what());
	}
}

void SearchBridge::queueMqttCommand(const struct mosquitto_message & message)
{
	if (message.topic == nullptr || _command_topic != message.topic) {
		return;
	}
	if (message.retain) {
		RCLCPP_WARN(
			get_logger(),
			"Ignoring a retained planner command; command topic must never be retained");
		return;
	}
	if (message.payloadlen < 0 ||
		(message.payloadlen > 0 && message.payload == nullptr)) {
		return;
	}

	const char * payload = static_cast<const char *>(message.payload);
	const std::string command_payload(
		payload == nullptr ? "" : payload,
		static_cast<std::size_t>(message.payloadlen));
	std::lock_guard<std::mutex> lock(_command_mutex);
	if (_command_queue.size() >= kMaximumQueuedCommands) {
		RCLCPP_WARN(get_logger(), "Dropping planner command because the command queue is full");
		return;
	}
	_command_queue.push_back(command_payload);
}

void SearchBridge::mqttConnectCallback(struct mosquitto * mosq, void * userdata, int result)
{
	if (userdata == nullptr) {
		return;
	}
	SearchBridge * self = static_cast<SearchBridge *>(userdata);
	if (result != MOSQ_ERR_SUCCESS) {
		self->_mqtt_connected.store(false);
		RCLCPP_WARN(
			self->get_logger(), "MQTT connection failed: %s", mosquitto_strerror(result));
		return;
	}

	const int subscribe_result = mosquitto_subscribe(
		mosq, nullptr, self->_command_topic.c_str(), 1);
	if (subscribe_result != MOSQ_ERR_SUCCESS) {
		self->_mqtt_connected.store(false);
		RCLCPP_ERROR(
			self->get_logger(),
			"MQTT planner command subscription failed: %s",
			mosquitto_strerror(subscribe_result));
		return;
	}
	self->_mqtt_connected.store(true);
	self->_publish_initial_state.store(true);
	RCLCPP_INFO(self->get_logger(), "Connected to MQTT broker and subscribed to '%s'", self->_command_topic.c_str());
}

void SearchBridge::mqttDisconnectCallback(struct mosquitto *, void * userdata, int result)
{
	if (userdata == nullptr) {
		return;
	}
	SearchBridge * self = static_cast<SearchBridge *>(userdata);
	self->_mqtt_connected.store(false);
	self->clearQueuedCommands();
	if (result != MOSQ_ERR_SUCCESS) {
		RCLCPP_WARN(self->get_logger(), "MQTT disconnected: %s", mosquitto_strerror(result));
	}
}

void SearchBridge::mqttMessageCallback(
	struct mosquitto *,
	void * userdata,
	const struct mosquitto_message * message)
{
	if (userdata == nullptr || message == nullptr) {
		return;
	}
	SearchBridge * self = static_cast<SearchBridge *>(userdata);
	self->queueMqttCommand(*message);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SearchBridge>());
	rclcpp::shutdown();
	return 0;
}
