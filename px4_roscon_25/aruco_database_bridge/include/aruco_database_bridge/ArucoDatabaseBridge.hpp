#pragma once

#include <aruco_database/msg/aruco_database_status.hpp>
#include <aruco_database/msg/aruco_marker_array.hpp>
#include <aruco_database/srv/clear_aruco_database.hpp>
#include <aruco_database/srv/get_aruco_position.hpp>
#include <aruco_database/srv/list_aruco_markers.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mosquitto.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

/**
 * @brief ROS 2 to MQTT bridge for the ArUco database management API.
 *
 * The bridge keeps the database node as the source of truth. It forwards
 * retained marker/status snapshots to MQTT and queues MQTT commands so that
 * ROS service calls run on the ROS executor thread.
 */
class ArucoDatabaseBridge final : public rclcpp::Node
{
public:
	explicit ArucoDatabaseBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
	~ArucoDatabaseBridge() override;

private:
	using SteadyClock = std::chrono::steady_clock;

	struct ParsedCommand
	{
		std::string request_id;
		std::string command;
		bool has_confirm{false};
		bool confirm{false};
		bool has_backup{false};
		bool backup{false};
		bool has_expected_revision{false};
		bool use_expected_revision{false};
		std::uint64_t expected_revision{0U};
		bool has_aruco_id{false};
		std::int32_t aruco_id{0};
	};

	void loadParameters();
	void configureTopics();
	void configureMqtt();
	void cleanupMqtt();

	void markersCallback(const aruco_database::msg::ArucoMarkerArray::SharedPtr message);
	void statusCallback(const aruco_database::msg::ArucoDatabaseStatus::SharedPtr message);
	void processTimer();
	void processQueuedCommands();
	void handleCommand(const std::string & payload);
	void requestSnapshot(const std::string & request_id, const std::string & command);
	void requestPosition(const std::string & request_id, std::int32_t aruco_id);
	void requestClear(const std::string & request_id, const ParsedCommand & command);

	static ParsedCommand parseCommandPayload(const std::string & payload);
	bool isRequestProcessed(const std::string & request_id) const;
	bool isRequestPending(const std::string & request_id) const;
	void markRequestPending(const std::string & request_id);
	void completeRequest(const std::string & request_id, const std::string & response);
	void clearQueuedCommands();

	std::string markersPayload() const;
	std::string statusPayload() const;
	std::string offlineStatusPayload() const;
	static std::string eventPayload(const std::string & event, const std::string & message);
	void publishMarkers();
	void publishStatus();
	void publishEvent(const std::string & event, const std::string & message);
	void publishResponse(
		const std::string & request_id,
		const std::string & command,
		bool success,
		const std::string & message,
		const std::string & extra_fields = "{}",
		bool track_request = true);
	void publishJson(
		const std::string & topic,
		const std::string & payload,
		int qos,
		bool retain);

	static std::string jsonString(const std::string & value);
	static std::int64_t unixTimeMilliseconds();

	void queueMqttCommand(const struct mosquitto_message & message);
	static void mqttConnectCallback(struct mosquitto * mosq, void * userdata, int result);
	static void mqttDisconnectCallback(struct mosquitto * mosq, void * userdata, int result);
	static void mqttMessageCallback(
		struct mosquitto * mosq,
		void * userdata,
		const struct mosquitto_message * message);

	std::shared_ptr<rclcpp::Subscription<aruco_database::msg::ArucoMarkerArray>>
		_markers_sub;
	std::shared_ptr<rclcpp::Subscription<aruco_database::msg::ArucoDatabaseStatus>>
		_status_sub;
	rclcpp::Client<aruco_database::srv::ListArucoMarkers>::SharedPtr _list_client;
	rclcpp::Client<aruco_database::srv::GetArucoPosition>::SharedPtr _position_client;
	rclcpp::Client<aruco_database::srv::ClearArucoDatabase>::SharedPtr _clear_client;
	rclcpp::TimerBase::SharedPtr _timer;

	mutable std::mutex _state_mutex;
	std::optional<aruco_database::msg::ArucoMarkerArray> _latest_markers;
	std::optional<aruco_database::msg::ArucoDatabaseStatus> _latest_status;
	SteadyClock::time_point _last_core_status{};

	mutable std::mutex _command_mutex;
	std::deque<std::string> _command_queue;
	std::unordered_map<std::string, std::string> _processed_responses;
	std::deque<std::string> _processed_request_order;
	std::unordered_set<std::string> _pending_requests;

	std::mutex _mqtt_mutex;
	struct mosquitto * _mosquitto{nullptr};
	bool _mosquitto_initialized{false};
	bool _mqtt_loop_started{false};
	std::atomic_bool _mqtt_connected{false};
	std::atomic_bool _publish_initial_state{false};
	std::atomic_bool _snapshot_request_pending{false};
	std::atomic<std::uint64_t> _mqtt_generation{0U};

	std::string _mqtt_host;
	int _mqtt_port{8883};
	std::string _mqtt_username;
	std::string _mqtt_password;
	std::string _mqtt_client_id{"aruco_database_bridge"};
	std::string _mqtt_topic_prefix{"aruco_database"};
	std::string _mqtt_tls_ca_file;
	std::string _markers_topic;
	std::string _status_topic;
	std::string _event_topic;
	std::string _command_topic;
	std::string _response_topic;
	int _status_publish_period_ms{1000};
	int _core_status_timeout_ms{5000};
	SteadyClock::time_point _last_status_publish{};
};
