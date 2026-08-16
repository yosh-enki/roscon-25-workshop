#include "aruco_database_bridge/ArucoDatabaseBridge.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{
constexpr std::size_t kMaximumQueuedCommands = 64U;
constexpr std::size_t kMaximumProcessedRequestIds = 256U;

const char * environmentValue(const char * name)
{
	const char * value = std::getenv(name);
	return value == nullptr ? "" : value;
}

void skipJsonWhitespace(const std::string & text, std::size_t & position)
{
	while (position < text.size() &&
		std::isspace(static_cast<unsigned char>(text[position])) != 0) {
		++position;
	}
}

void expectJsonCharacter(const std::string & text, std::size_t & position, const char expected)
{
	skipJsonWhitespace(text, position);
	if (position >= text.size() || text[position] != expected) {
		throw std::runtime_error(std::string("expected JSON character '") + expected + "'");
	}
	++position;
}

std::string parseJsonString(const std::string & text, std::size_t & position)
{
	skipJsonWhitespace(text, position);
	if (position >= text.size() || text[position] != '"') {
		throw std::runtime_error("expected JSON string");
	}
	++position;

	std::string result;
	while (position < text.size()) {
		const char character = text[position++];
		if (character == '"') {
			return result;
		}
		if (static_cast<unsigned char>(character) < 0x20U) {
			throw std::runtime_error("JSON string contains a control character");
		}
		if (character != '\\') {
			result.push_back(character);
			continue;
		}
		if (position >= text.size()) {
			throw std::runtime_error("JSON string has an incomplete escape");
		}
		switch (text[position++]) {
		case '"': result.push_back('"'); break;
		case '\\': result.push_back('\\'); break;
		case '/': result.push_back('/'); break;
		case 'b': result.push_back('\b'); break;
		case 'f': result.push_back('\f'); break;
		case 'n': result.push_back('\n'); break;
		case 'r': result.push_back('\r'); break;
		case 't': result.push_back('\t'); break;
		default:
			throw std::runtime_error("JSON string contains an unsupported escape");
		}
	}
	throw std::runtime_error("JSON string is unterminated");
}

void skipJsonValue(const std::string & text, std::size_t & position)
{
	skipJsonWhitespace(text, position);
	if (position >= text.size()) {
		throw std::runtime_error("JSON value is missing");
	}

	if (text[position] == '"') {
		(void)parseJsonString(text, position);
		return;
	}

	if (text[position] == '{' || text[position] == '[') {
		const char opening = text[position++];
		const char closing = opening == '{' ? '}' : ']';
		skipJsonWhitespace(text, position);
		if (position < text.size() && text[position] == closing) {
			++position;
			return;
		}
		while (true) {
			if (opening == '{') {
				(void)parseJsonString(text, position);
				expectJsonCharacter(text, position, ':');
			}
			skipJsonValue(text, position);
			skipJsonWhitespace(text, position);
			if (position < text.size() && text[position] == ',') {
				++position;
				continue;
			}
			if (position < text.size() && text[position] == closing) {
				++position;
				return;
			}
			throw std::runtime_error("malformed JSON collection");
		}
	}

	const std::size_t start = position;
	while (position < text.size() &&
		!std::isspace(static_cast<unsigned char>(text[position])) &&
		text[position] != ',' && text[position] != '}' && text[position] != ']') {
		++position;
	}
	if (position == start) {
		throw std::runtime_error("malformed JSON value");
	}
}

bool parseJsonBool(const std::string & text, std::size_t & position)
{
	skipJsonWhitespace(text, position);
	if (text.compare(position, 4U, "true") == 0) {
		position += 4U;
		return true;
	}
	if (text.compare(position, 5U, "false") == 0) {
		position += 5U;
		return false;
	}
	throw std::runtime_error("expected JSON boolean");
}

std::uint64_t parseJsonUnsigned(const std::string & text, std::size_t & position)
{
	skipJsonWhitespace(text, position);
	const std::size_t start = position;
	if (position < text.size() && text[position] == '-') {
		throw std::runtime_error("expected a non-negative JSON integer");
	}
	while (position < text.size() && std::isdigit(static_cast<unsigned char>(text[position])) != 0) {
		++position;
	}
	if (position == start) {
		throw std::runtime_error("expected a JSON integer");
	}
	try {
		return std::stoull(text.substr(start, position - start));
	} catch (const std::exception &) {
		throw std::runtime_error("JSON integer is out of range");
	}
}

std::int32_t parseJsonInt32(const std::string & text, std::size_t & position)
{
	skipJsonWhitespace(text, position);
	const std::size_t start = position;
	if (position < text.size() && (text[position] == '-' || text[position] == '+')) {
		++position;
	}
	while (position < text.size() && std::isdigit(static_cast<unsigned char>(text[position])) != 0) {
		++position;
	}
	if (position == start ||
		(position == start + 1U && (text[start] == '-' || text[start] == '+'))) {
		throw std::runtime_error("expected a JSON integer");
	}
	try {
		const long long value = std::stoll(text.substr(start, position - start));
		if (value < std::numeric_limits<std::int32_t>::min() ||
			value > std::numeric_limits<std::int32_t>::max()) {
			throw std::runtime_error("JSON integer is outside int32 range");
		}
		return static_cast<std::int32_t>(value);
	} catch (const std::invalid_argument &) {
		throw std::runtime_error("invalid JSON integer");
	} catch (const std::out_of_range &) {
		throw std::runtime_error("JSON integer is out of range");
	}
}

std::string formatDouble(const double value)
{
	std::ostringstream output;
	output << std::setprecision(17) << value;
	return output.str();
}

}  // namespace

ArucoDatabaseBridge::ArucoDatabaseBridge(const rclcpp::NodeOptions & options)
	: Node("aruco_database_bridge", options)
{
	loadParameters();
	configureTopics();

	const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
	_markers_sub = create_subscription<aruco_database::msg::ArucoMarkerArray>(
		"/aruco_database/markers",
		snapshot_qos,
		std::bind(&ArucoDatabaseBridge::markersCallback, this, std::placeholders::_1));
	_status_sub = create_subscription<aruco_database::msg::ArucoDatabaseStatus>(
		"/aruco_database/status",
		snapshot_qos,
		std::bind(&ArucoDatabaseBridge::statusCallback, this, std::placeholders::_1));
	_list_client = create_client<aruco_database::srv::ListArucoMarkers>(
		"/aruco_database/list_markers");
	_position_client = create_client<aruco_database::srv::GetArucoPosition>(
		"/aruco_database/get_position");
	_clear_client = create_client<aruco_database::srv::ClearArucoDatabase>(
		"/aruco_database/clear");

	_timer = create_wall_timer(
		std::chrono::milliseconds(std::min(_status_publish_period_ms, 1000)),
		std::bind(&ArucoDatabaseBridge::processTimer, this));

	configureMqtt();
	RCLCPP_INFO(
		get_logger(),
		"aruco_database_bridge initialized: markers='%s', status='%s', command='%s'",
		_markers_topic.c_str(), _status_topic.c_str(), _command_topic.c_str());
}

ArucoDatabaseBridge::~ArucoDatabaseBridge()
{
	cleanupMqtt();
}

void ArucoDatabaseBridge::loadParameters()
{
	declare_parameter<std::string>("mqtt_host", "");
	declare_parameter<int>("mqtt_port", 8883);
	declare_parameter<std::string>("mqtt_username", "");
	declare_parameter<std::string>("mqtt_password", "");
	declare_parameter<std::string>("mqtt_client_id", "aruco_database_bridge");
	declare_parameter<std::string>("mqtt_topic_prefix", "aruco_database");
	declare_parameter<std::string>("mqtt_tls_ca_file", "");
	declare_parameter<int>("status_publish_period_ms", 1000);
	declare_parameter<int>("core_status_timeout_ms", 5000);

	get_parameter("mqtt_host", _mqtt_host);
	get_parameter("mqtt_port", _mqtt_port);
	get_parameter("mqtt_username", _mqtt_username);
	get_parameter("mqtt_password", _mqtt_password);
	get_parameter("mqtt_client_id", _mqtt_client_id);
	get_parameter("mqtt_topic_prefix", _mqtt_topic_prefix);
	get_parameter("mqtt_tls_ca_file", _mqtt_tls_ca_file);
	get_parameter("status_publish_period_ms", _status_publish_period_ms);
	get_parameter("core_status_timeout_ms", _core_status_timeout_ms);

	if (_mqtt_username.empty()) {
		_mqtt_username = environmentValue("HIVEMQ_MQTT_USERNAME");
	}
	if (_mqtt_password.empty()) {
		_mqtt_password = environmentValue("HIVEMQ_MQTT_PASSWORD");
	}

	if (_mqtt_port < 1 || _mqtt_port > 65535) {
		throw std::invalid_argument(
			"aruco_database_bridge: mqtt_port must be between 1 and 65535");
	}
	if (_status_publish_period_ms < 100) {
		throw std::invalid_argument(
			"aruco_database_bridge: status_publish_period_ms must be at least 100 milliseconds");
	}
	if (_core_status_timeout_ms < 500) {
		throw std::invalid_argument(
			"aruco_database_bridge: core_status_timeout_ms must be at least 500 milliseconds");
	}
	if (_mqtt_client_id.empty()) {
		throw std::invalid_argument("aruco_database_bridge: mqtt_client_id must not be empty");
	}
	if (_mqtt_username.empty() != _mqtt_password.empty()) {
		throw std::invalid_argument(
			"aruco_database_bridge: provide both MQTT credentials or leave both empty");
	}
	if (!_mqtt_host.empty() && (_mqtt_username.empty() || _mqtt_password.empty())) {
		throw std::invalid_argument(
			"aruco_database_bridge: MQTT credentials are required when mqtt_host is configured");
	}
}

void ArucoDatabaseBridge::configureTopics()
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
			"aruco_database_bridge: mqtt_topic_prefix must be a non-empty concrete prefix");
	}

	_markers_topic = _mqtt_topic_prefix + "/markers";
	_status_topic = _mqtt_topic_prefix + "/status";
	_event_topic = _mqtt_topic_prefix + "/event";
	_command_topic = _mqtt_topic_prefix + "/command";
	_response_topic = _mqtt_topic_prefix + "/response";
}

void ArucoDatabaseBridge::configureMqtt()
{
	if (_mqtt_host.empty()) {
		RCLCPP_WARN(
			get_logger(),
			"mqtt_host is empty; aruco_database_bridge will run ROS subscriptions "
			"but will not connect to MQTT");
		return;
	}

	int result = mosquitto_lib_init();
	if (result != MOSQ_ERR_SUCCESS) {
		throw std::runtime_error(
			std::string("aruco_database_bridge: mosquitto_lib_init failed: ") +
			mosquitto_strerror(result));
	}
	_mosquitto_initialized = true;

	_mosquitto = mosquitto_new(_mqtt_client_id.c_str(), true, this);
	if (_mosquitto == nullptr) {
		cleanupMqtt();
		throw std::runtime_error("aruco_database_bridge: cannot create MQTT client");
	}

	mosquitto_connect_callback_set(_mosquitto, &ArucoDatabaseBridge::mqttConnectCallback);
	mosquitto_disconnect_callback_set(_mosquitto, &ArucoDatabaseBridge::mqttDisconnectCallback);
	mosquitto_message_callback_set(_mosquitto, &ArucoDatabaseBridge::mqttMessageCallback);
	(void)mosquitto_reconnect_delay_set(_mosquitto, 2U, 30U, true);

	const std::string offline_status = offlineStatusPayload();
	result = mosquitto_will_set(
		_mosquitto,
		_status_topic.c_str(),
		static_cast<int>(offline_status.size()),
		offline_status.data(),
		1,
		true);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string error =
			std::string("aruco_database_bridge: MQTT Last Will setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(error);
	}

	const char * ca_file = _mqtt_tls_ca_file.empty() ? nullptr : _mqtt_tls_ca_file.c_str();
	const char * ca_path = _mqtt_tls_ca_file.empty() ? "/etc/ssl/certs" : nullptr;
	result = mosquitto_tls_set(_mosquitto, ca_file, ca_path, nullptr, nullptr, nullptr);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string error =
			std::string("aruco_database_bridge: MQTT TLS setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(error);
	}
	result = mosquitto_tls_insecure_set(_mosquitto, false);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string error =
			std::string("aruco_database_bridge: MQTT certificate verification setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(error);
	}

	if (!_mqtt_username.empty()) {
		result = mosquitto_username_pw_set(
			_mosquitto, _mqtt_username.c_str(), _mqtt_password.c_str());
		if (result != MOSQ_ERR_SUCCESS) {
			const std::string error =
				std::string("aruco_database_bridge: MQTT credential setup failed: ") +
				mosquitto_strerror(result);
			cleanupMqtt();
			throw std::runtime_error(error);
		}
	}

	result = mosquitto_connect_async(_mosquitto, _mqtt_host.c_str(), _mqtt_port, 60);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string error =
			std::string("aruco_database_bridge: MQTT connection setup failed: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(error);
	}

	result = mosquitto_loop_start(_mosquitto);
	if (result != MOSQ_ERR_SUCCESS) {
		const std::string error =
			std::string("aruco_database_bridge: MQTT network loop failed to start: ") +
			mosquitto_strerror(result);
		cleanupMqtt();
		throw std::runtime_error(error);
	}
	_mqtt_loop_started = true;
	RCLCPP_INFO(
		get_logger(), "Connecting to MQTT broker '%s:%d' with TLS",
		_mqtt_host.c_str(), _mqtt_port);
}

void ArucoDatabaseBridge::cleanupMqtt()
{
	std::lock_guard<std::mutex> lock(_mqtt_mutex);
	if (_mosquitto != nullptr && _mqtt_connected.load()) {
		const std::string offline_status = offlineStatusPayload();
		(void)mosquitto_publish(
			_mosquitto,
			nullptr,
			_status_topic.c_str(),
			static_cast<int>(offline_status.size()),
			offline_status.data(),
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

void ArucoDatabaseBridge::markersCallback(
	const aruco_database::msg::ArucoMarkerArray::SharedPtr message)
{
	{
		std::lock_guard<std::mutex> lock(_state_mutex);
		_latest_markers = *message;
	}
	publishMarkers();
}

void ArucoDatabaseBridge::statusCallback(
	const aruco_database::msg::ArucoDatabaseStatus::SharedPtr message)
{
	bool persistence_failed = false;
	bool persistence_recovered = false;
	std::string persistence_message;
	{
		std::lock_guard<std::mutex> lock(_state_mutex);
		if (_latest_status.has_value()) {
			persistence_failed = _latest_status->persistence_ok && !message->persistence_ok;
			persistence_recovered = !_latest_status->persistence_ok && message->persistence_ok;
		}
		_latest_status = *message;
		_last_core_status = SteadyClock::now();
		persistence_message = message->last_error;
	}
	publishStatus();
	if (persistence_failed) {
		publishEvent("persistence_error", persistence_message);
	} else if (persistence_recovered) {
		publishEvent("persistence_recovered", "ArUco database persistence recovered");
	}
}

void ArucoDatabaseBridge::processTimer()
{
	if (!_mqtt_connected.load()) {
		return;
	}

	if (_publish_initial_state.exchange(false)) {
		publishMarkers();
		publishStatus();
		requestSnapshot("", "initial_refresh");
	}

	processQueuedCommands();

	const auto now = SteadyClock::now();
	if (_last_status_publish.time_since_epoch().count() == 0 ||
		std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_status_publish).count() >=
		_status_publish_period_ms) {
		publishStatus();
		_last_status_publish = now;
	}
}

void ArucoDatabaseBridge::processQueuedCommands()
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

ArucoDatabaseBridge::ParsedCommand ArucoDatabaseBridge::parseCommandPayload(
	const std::string & payload)
{
	ParsedCommand command;
	std::size_t position = 0U;
	skipJsonWhitespace(payload, position);
	expectJsonCharacter(payload, position, '{');
	skipJsonWhitespace(payload, position);
	if (position < payload.size() && payload[position] == '}') {
		throw std::runtime_error("command JSON object is empty");
	}

	while (true) {
		const std::string key = parseJsonString(payload, position);
		expectJsonCharacter(payload, position, ':');
		if (key == "request_id") {
			command.request_id = parseJsonString(payload, position);
		} else if (key == "command") {
			command.command = parseJsonString(payload, position);
		} else if (key == "confirm") {
			command.has_confirm = true;
			command.confirm = parseJsonBool(payload, position);
		} else if (key == "backup") {
			command.has_backup = true;
			command.backup = parseJsonBool(payload, position);
		} else if (key == "use_expected_revision") {
			command.use_expected_revision = parseJsonBool(payload, position);
		} else if (key == "expected_revision") {
			command.has_expected_revision = true;
			command.expected_revision = parseJsonUnsigned(payload, position);
		} else if (key == "aruco_id") {
			command.has_aruco_id = true;
			command.aruco_id = parseJsonInt32(payload, position);
		} else {
			skipJsonValue(payload, position);
		}

		skipJsonWhitespace(payload, position);
		if (position < payload.size() && payload[position] == ',') {
			++position;
			continue;
		}
		if (position < payload.size() && payload[position] == '}') {
			++position;
			break;
		}
		throw std::runtime_error("command JSON object is malformed");
	}

	skipJsonWhitespace(payload, position);
	if (position != payload.size()) {
		throw std::runtime_error("command JSON has trailing characters");
	}
	if (command.request_id.empty()) {
		throw std::runtime_error("request_id must be a non-empty string");
	}
	if (command.command.empty()) {
		throw std::runtime_error("command must be a non-empty string");
	}
	if (command.use_expected_revision && !command.has_expected_revision) {
		throw std::runtime_error("expected_revision is required when use_expected_revision is true");
	}
	return command;
}

void ArucoDatabaseBridge::handleCommand(const std::string & payload)
{
	ParsedCommand command;
	try {
		command = parseCommandPayload(payload);
	} catch (const std::exception & error) {
		publishResponse("", "", false, error.what());
		return;
	}

	if (isRequestProcessed(command.request_id)) {
		std::string cached_response;
		{
			std::lock_guard<std::mutex> lock(_command_mutex);
			const auto iterator = _processed_responses.find(command.request_id);
			if (iterator != _processed_responses.end()) {
				cached_response = iterator->second;
			}
		}
		if (!cached_response.empty()) {
			publishJson(_response_topic, cached_response, 1, false);
		}
		return;
	}
	if (isRequestPending(command.request_id)) {
		publishResponse(
			command.request_id, command.command, false,
			"request_id is already being processed", "{}", false);
		return;
	}
	markRequestPending(command.request_id);

	if (command.command == "status") {
		publishStatus();
		publishResponse(command.request_id, command.command, true, "Status published");
		return;
	}
	if (command.command == "refresh" || command.command == "list") {
		requestSnapshot(command.request_id, command.command);
		return;
	}
	if (command.command == "get_marker") {
		if (!command.has_aruco_id) {
			publishResponse(command.request_id, command.command, false, "aruco_id is required");
			return;
		}
		requestPosition(command.request_id, command.aruco_id);
		return;
	}
	if (command.command == "clear_file") {
		requestClear(command.request_id, command);
		return;
	}

	publishResponse(
		command.request_id, command.command, false,
		"Unsupported command. Use status, refresh, get_marker, or clear_file");
}

void ArucoDatabaseBridge::requestSnapshot(
	const std::string & request_id, const std::string & command)
{
	if (_snapshot_request_pending.exchange(true)) {
		if (!request_id.empty()) {
			publishResponse(request_id, command, false, "A snapshot request is already pending");
		}
		return;
	}

	if (!_list_client->service_is_ready()) {
		_snapshot_request_pending.store(false);
		if (!request_id.empty()) {
			publishResponse(request_id, command, false, "aruco_database list service is unavailable");
		} else {
			publishEvent("core_unavailable", "aruco_database list service is unavailable");
		}
		return;
	}

	auto request = std::make_shared<aruco_database::srv::ListArucoMarkers::Request>();
	const auto request_generation = _mqtt_generation.load();
	_list_client->async_send_request(
		request,
		[this, request_id, command, request_generation](
			rclcpp::Client<aruco_database::srv::ListArucoMarkers>::SharedFuture future) {
			if (_mqtt_generation.load() == request_generation) {
				_snapshot_request_pending.store(false);
			}
			try {
				const auto response = future.get();
				if (!response->success) {
					if (!request_id.empty()) {
						publishResponse(request_id, command, false, response->error_message);
					}
					return;
				}

			aruco_database::msg::ArucoMarkerArray snapshot;
			snapshot.header.stamp = now();
			snapshot.revision = response->revision;
			snapshot.markers = response->markers;
			{
				std::lock_guard<std::mutex> lock(_state_mutex);
				_latest_markers = snapshot;
			}
			publishMarkers();
			if (!request_id.empty()) {
				publishResponse(
					request_id, command, true, "Marker snapshot published",
					"{\"revision\":" + std::to_string(response->revision) +
					",\"marker_count\":" + std::to_string(response->markers.size()) + "}");
			} else {
				publishEvent("snapshot_refreshed", "Marker snapshot refreshed from aruco_database");
			}
			} catch (const std::exception & error) {
				if (!request_id.empty()) {
					publishResponse(request_id, command, false, error.what());
				} else {
					publishEvent("core_error", error.what());
				}
			}
		});
}

void ArucoDatabaseBridge::requestPosition(
	const std::string & request_id, const std::int32_t aruco_id)
{
	if (!_position_client->service_is_ready()) {
		publishResponse(request_id, "get_marker", false, "aruco_database position service is unavailable");
		return;
	}

	auto request = std::make_shared<aruco_database::srv::GetArucoPosition::Request>();
	request->aruco_id = aruco_id;
	_position_client->async_send_request(
		request,
		[this, request_id, aruco_id](
			rclcpp::Client<aruco_database::srv::GetArucoPosition>::SharedFuture future) {
			try {
				const auto response = future.get();
				const std::string extra =
					"{\"found\":" + std::string(response->found ? "true" : "false") +
					",\"aruco_id\":" + std::to_string(aruco_id) +
					(response->found
						? ",\"latitude_deg\":" + formatDouble(response->latitude_deg) +
						  ",\"longitude_deg\":" + formatDouble(response->longitude_deg)
						: "") + "}";
				publishResponse(
					request_id, "get_marker", response->found,
					response->found ? "Marker found" : response->error_message, extra);
			} catch (const std::exception & error) {
				publishResponse(request_id, "get_marker", false, error.what());
			}
		});
}

void ArucoDatabaseBridge::requestClear(
	const std::string & request_id, const ParsedCommand & command)
{
	if (!command.has_confirm || !command.confirm) {
		publishResponse(request_id, "clear_file", false, "confirm must be true");
		return;
	}
	if (!_clear_client->service_is_ready()) {
		publishResponse(request_id, "clear_file", false, "aruco_database clear service is unavailable");
		return;
	}

	auto request = std::make_shared<aruco_database::srv::ClearArucoDatabase::Request>();
	request->confirm = true;
	request->backup = command.has_backup && command.backup;
	request->use_expected_revision = command.use_expected_revision || command.has_expected_revision;
	request->expected_revision = command.expected_revision;
	_clear_client->async_send_request(
		request,
		[this, request_id](
			rclcpp::Client<aruco_database::srv::ClearArucoDatabase>::SharedFuture future) {
			try {
				const auto response = future.get();
				const std::string extra =
					"{\"revision\":" + std::to_string(response->revision) +
					",\"marker_count\":" + std::to_string(response->marker_count) +
					",\"persisted\":" + std::string(response->persisted ? "true" : "false") +
					",\"backup_file\":" + jsonString(response->backup_file) + "}";
				publishResponse(
					request_id, "clear_file", response->success,
					response->success ? "Database cleared successfully" : response->error_message,
					extra);
				publishEvent(
					response->success ? "database_cleared" : "database_clear_failed",
					response->success ? "Persistent database cleared" : response->error_message);
			} catch (const std::exception & error) {
				publishResponse(request_id, "clear_file", false, error.what());
				publishEvent("database_clear_failed", error.what());
			}
		});
}

bool ArucoDatabaseBridge::isRequestProcessed(const std::string & request_id) const
{
	std::lock_guard<std::mutex> lock(_command_mutex);
	return _processed_responses.find(request_id) != _processed_responses.end();
}

bool ArucoDatabaseBridge::isRequestPending(const std::string & request_id) const
{
	std::lock_guard<std::mutex> lock(_command_mutex);
	return _pending_requests.find(request_id) != _pending_requests.end();
}

void ArucoDatabaseBridge::markRequestPending(const std::string & request_id)
{
	std::lock_guard<std::mutex> lock(_command_mutex);
	_pending_requests.insert(request_id);
}

void ArucoDatabaseBridge::completeRequest(
	const std::string & request_id, const std::string & response)
{
	{
		std::lock_guard<std::mutex> lock(_command_mutex);
		_pending_requests.erase(request_id);
		if (!request_id.empty()) {
			if (_processed_responses.find(request_id) == _processed_responses.end()) {
				_processed_request_order.push_back(request_id);
			}
			_processed_responses[request_id] = response;
			while (_processed_request_order.size() > kMaximumProcessedRequestIds) {
				_processed_responses.erase(_processed_request_order.front());
				_processed_request_order.pop_front();
			}
		}
	}
	publishJson(_response_topic, response, 1, false);
}

void ArucoDatabaseBridge::clearQueuedCommands()
{
	std::lock_guard<std::mutex> lock(_command_mutex);
	_command_queue.clear();
}

std::string ArucoDatabaseBridge::markersPayload() const
{
	std::optional<aruco_database::msg::ArucoMarkerArray> markers;
	SteadyClock::time_point last_core_status;
	{
		std::lock_guard<std::mutex> lock(_state_mutex);
		markers = _latest_markers;
		last_core_status = _last_core_status;
	}

	const auto now = SteadyClock::now();
	const bool core_available = markers.has_value() &&
		last_core_status.time_since_epoch().count() != 0 && now >= last_core_status &&
		std::chrono::duration_cast<std::chrono::milliseconds>(now - last_core_status).count() <=
		_core_status_timeout_ms;
	const std::uint64_t revision = markers.has_value() ? markers->revision : 0U;
	std::ostringstream output;
	output << "{\"schema\":\"aruco_database.markers.v1\""
		   << ",\"source\":\"aruco_database_bridge\""
		   << ",\"published_at_unix_ms\":" << unixTimeMilliseconds()
		   << ",\"core_available\":" << (core_available ? "true" : "false")
		   << ",\"revision\":" << revision
		   << ",\"markers\":[";
	if (markers.has_value()) {
		for (std::size_t index = 0; index < markers->markers.size(); ++index) {
			const auto & marker = markers->markers[index];
			if (index != 0U) {
				output << ',';
			}
			output << "{\"id\":" << marker.id
				   << ",\"latitude_deg\":" << formatDouble(marker.latitude_deg)
				   << ",\"longitude_deg\":" << formatDouble(marker.longitude_deg)
				   << ",\"observation_count\":" << marker.observation_count << '}';
		}
	}
	output << "]}";
	return output.str();
}

std::string ArucoDatabaseBridge::statusPayload() const
{
	std::optional<aruco_database::msg::ArucoDatabaseStatus> status;
	std::optional<aruco_database::msg::ArucoMarkerArray> markers;
	SteadyClock::time_point last_core_status;
	{
		std::lock_guard<std::mutex> lock(_state_mutex);
		status = _latest_status;
		markers = _latest_markers;
		last_core_status = _last_core_status;
	}

	const auto now = SteadyClock::now();
	const bool core_available = status.has_value() &&
		last_core_status.time_since_epoch().count() != 0 && now >= last_core_status &&
		std::chrono::duration_cast<std::chrono::milliseconds>(now - last_core_status).count() <=
		_core_status_timeout_ms;

	std::ostringstream output;
	output << "{\"schema\":\"aruco_database.status.v1\""
		   << ",\"source\":\"aruco_database_bridge\""
		   << ",\"published_at_unix_ms\":" << unixTimeMilliseconds()
		   << ",\"bridge_online\":" << (_mqtt_connected.load() ? "true" : "false")
		   << ",\"core_available\":" << (core_available ? "true" : "false");
	if (core_available) {
		std::string state = "ready";
		if (!status->persistence_ok) {
			state = "persistence_error";
		} else if (!status->origin_ready) {
			state = "waiting_for_origin";
		} else if (status->marker_count == 0U) {
			state = "ready_empty";
		}
		output << ",\"state\":" << jsonString(state)
			   << ",\"revision\":" << status->revision
			   << ",\"marker_count\":" << status->marker_count
			   << ",\"origin_ready\":" << (status->origin_ready ? "true" : "false")
			   << ",\"database_dirty\":" << (status->database_dirty ? "true" : "false")
			   << ",\"persistence_ok\":" << (status->persistence_ok ? "true" : "false")
			   << ",\"file_state\":" << jsonString(status->persistence_state)
			   << ",\"last_error\":" << jsonString(status->last_error);
	} else {
		const std::uint64_t revision = status.has_value() ? status->revision : 0U;
		const std::size_t marker_count = status.has_value()
			? status->marker_count
			: (markers.has_value() ? markers->markers.size() : 0U);
		const std::string error = status.has_value()
			? "aruco_database status is stale"
			: "aruco_database status has not arrived";
		output << ",\"state\":\"core_unavailable\""
			   << ",\"revision\":" << revision
			   << ",\"marker_count\":" << marker_count
			   << ",\"origin_ready\":false"
			   << ",\"database_dirty\":false"
			   << ",\"persistence_ok\":false"
			   << ",\"file_state\":\"unknown\""
			   << ",\"last_error\":" << jsonString(error);
	}
	output << '}';
	return output.str();
}

std::string ArucoDatabaseBridge::offlineStatusPayload() const
{
	std::ostringstream output;
	output << "{\"schema\":\"aruco_database.status.v1\""
		   << ",\"source\":\"aruco_database_bridge\""
		   << ",\"published_at_unix_ms\":" << unixTimeMilliseconds()
		   << ",\"state\":\"offline\""
		   << ",\"bridge_online\":false"
		   << ",\"core_available\":false"
		   << ",\"persistence_ok\":false"
		   << ",\"file_state\":\"offline\""
		   << ",\"last_error\":\"MQTT bridge is offline\"}";
	return output.str();
}

std::string ArucoDatabaseBridge::eventPayload(
	const std::string & event, const std::string & message)
{
	return "{\"schema\":\"aruco_database.event.v1\",\"event\":" +
		jsonString(event) + ",\"message\":" + jsonString(message) +
		",\"published_at_unix_ms\":" + std::to_string(unixTimeMilliseconds()) + "}";
}

void ArucoDatabaseBridge::publishMarkers()
{
	if (_mqtt_connected.load()) {
		publishJson(_markers_topic, markersPayload(), 1, true);
	}
}

void ArucoDatabaseBridge::publishStatus()
{
	if (_mqtt_connected.load()) {
		publishJson(_status_topic, statusPayload(), 1, true);
	}
}

void ArucoDatabaseBridge::publishEvent(
	const std::string & event, const std::string & message)
{
	if (_mqtt_connected.load()) {
		publishJson(_event_topic, eventPayload(event, message), 1, false);
	}
}

void ArucoDatabaseBridge::publishResponse(
	const std::string & request_id,
	const std::string & command,
	const bool success,
	const std::string & message,
	const std::string & extra_fields,
	const bool track_request)
{
	std::string extra;
	if (extra_fields.size() >= 2U && extra_fields.front() == '{' && extra_fields.back() == '}') {
		extra = extra_fields.substr(1U, extra_fields.size() - 2U);
	}
	std::ostringstream output;
	output << "{\"schema\":\"aruco_database.response.v1\""
		   << ",\"request_id\":" << jsonString(request_id)
		   << ",\"command\":" << jsonString(command)
		   << ",\"success\":" << (success ? "true" : "false")
		   << ",\"message\":" << jsonString(message)
		   << ",\"responded_at_unix_ms\":" << unixTimeMilliseconds();
	if (!extra.empty()) {
		output << ',' << extra;
	}
	output << '}';
	if (track_request) {
		completeRequest(request_id, output.str());
	} else {
		publishJson(_response_topic, output.str(), 1, false);
	}
}

void ArucoDatabaseBridge::publishJson(
	const std::string & topic, const std::string & payload, const int qos, const bool retain)
{
	std::lock_guard<std::mutex> lock(_mqtt_mutex);
	if (_mosquitto == nullptr || !_mqtt_connected.load()) {
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
			get_logger(), "Could not publish MQTT topic '%s': %s",
			topic.c_str(), mosquitto_strerror(result));
	}
}

std::string ArucoDatabaseBridge::jsonString(const std::string & value)
{
	std::string escaped;
	escaped.reserve(value.size() + 2U);
	escaped.push_back('"');
	for (const unsigned char character : value) {
		switch (character) {
		case '"': escaped += "\\\""; break;
		case '\\': escaped += "\\\\"; break;
		case '\b': escaped += "\\b"; break;
		case '\f': escaped += "\\f"; break;
		case '\n': escaped += "\\n"; break;
		case '\r': escaped += "\\r"; break;
		case '\t': escaped += "\\t"; break;
		default:
			if (character < 0x20U) {
				std::ostringstream code;
				code << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
					 << static_cast<unsigned int>(character);
				escaped += code.str();
			} else {
				escaped.push_back(static_cast<char>(character));
			}
		}
	}
	escaped.push_back('"');
	return escaped;
}

std::int64_t ArucoDatabaseBridge::unixTimeMilliseconds()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
}

void ArucoDatabaseBridge::queueMqttCommand(const struct mosquitto_message & message)
{
	if (message.topic == nullptr || _command_topic != message.topic) {
		return;
	}
	if (message.retain) {
		RCLCPP_WARN(
			get_logger(), "Ignoring retained MQTT command; the command topic must never be retained");
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
		RCLCPP_WARN(get_logger(), "Dropping MQTT command because the command queue is full");
		return;
	}
	_command_queue.push_back(command_payload);
}

void ArucoDatabaseBridge::mqttConnectCallback(
	struct mosquitto * mosq, void * userdata, const int result)
{
	if (userdata == nullptr) {
		return;
	}
	ArucoDatabaseBridge * self = static_cast<ArucoDatabaseBridge *>(userdata);
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
			self->get_logger(), "MQTT command subscription failed: %s",
			mosquitto_strerror(subscribe_result));
		return;
	}
	self->_mqtt_connected.store(true);
	self->_mqtt_generation.fetch_add(1U);
	self->_publish_initial_state.store(true);
	RCLCPP_INFO(
		self->get_logger(), "Connected to MQTT and subscribed to '%s'",
		self->_command_topic.c_str());
}

void ArucoDatabaseBridge::mqttDisconnectCallback(
	struct mosquitto *, void * userdata, const int result)
{
	if (userdata == nullptr) {
		return;
	}
	ArucoDatabaseBridge * self = static_cast<ArucoDatabaseBridge *>(userdata);
	self->_mqtt_connected.store(false);
	self->_mqtt_generation.fetch_add(1U);
	self->_snapshot_request_pending.store(false);
	self->clearQueuedCommands();
	if (result != MOSQ_ERR_SUCCESS) {
		RCLCPP_WARN(
			self->get_logger(), "MQTT disconnected: %s", mosquitto_strerror(result));
	}
}

void ArucoDatabaseBridge::mqttMessageCallback(
	struct mosquitto *, void * userdata, const struct mosquitto_message * message)
{
	if (userdata == nullptr || message == nullptr) {
		return;
	}
	ArucoDatabaseBridge * self = static_cast<ArucoDatabaseBridge *>(userdata);
	self->queueMqttCommand(*message);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoDatabaseBridge>());
	rclcpp::shutdown();
	return 0;
}
