#pragma once

#include "search/SearchPlanner.hpp"

#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
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

/**
 * @brief ROS 2 to MQTT bridge for PX4 status and SearchPlanner controls.
 *
 * The MQTT callback only queues planner commands. All filesystem access and
 * SearchPlanner calls run on the ROS executor thread, where the latest PX4
 * VehicleStatus can be checked before every operation.
 */
class SearchBridge final : public rclcpp::Node
{
public:
	explicit SearchBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
	~SearchBridge() override;

private:
	using SteadyClock = std::chrono::steady_clock;

	struct SafetySnapshot
	{
		bool known{false};
		bool fresh{false};
		bool disarmed{false};
		std::uint8_t arming_state{0U};
		double age_ms{0.0};
	};

	void loadParameters();
	void resolvePlanDirectories();
	void configureTopics();
	void configureMqtt();
	void cleanupMqtt();

	void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr message);
	void vehicleLandDetectedCallback(
		const px4_msgs::msg::VehicleLandDetected::SharedPtr message);
	void processTimer();
	void processQueuedCommands();
	void handleCommand(const std::string & payload);
	bool isRequestIdProcessed(const std::string & request_id) const;
	void cacheResponse(const std::string & request_id, const std::string & payload);
	void clearQueuedCommands();

	SafetySnapshot safetySnapshot() const;
	std::string safetyFailureReason(const SafetySnapshot & safety) const;
	static std::string armingStateName(std::uint8_t arming_state);
	static std::int64_t unixTimeMilliseconds();
	std::string offlineStatusPayload() const;
	std::string offlinePlannerStatusPayload() const;

	std::string statusPayload();
	std::string plansPayload();
	std::string plannerStatusPayload() const;
	void publishStatus();
	void publishPlans();
	void publishPlannerStatus();
	void publishResponse(
		const std::string & request_id,
		bool success,
		const std::string & message,
		const std::optional<std::string> & active_working_plan = std::nullopt);
	void publishJson(
		const std::string & topic,
		const std::string & payload,
		int qos,
		bool retain);

	void queueMqttCommand(const struct mosquitto_message & message);
	static void mqttConnectCallback(struct mosquitto * mosq, void * userdata, int result);
	static void mqttDisconnectCallback(struct mosquitto * mosq, void * userdata, int result);
	static void mqttMessageCallback(
		struct mosquitto * mosq,
		void * userdata,
		const struct mosquitto_message * message);

	std::shared_ptr<rclcpp::Subscription<px4_msgs::msg::VehicleStatus>> _vehicle_status_sub;
	std::shared_ptr<rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>>
		_vehicle_land_detected_sub;
	rclcpp::TimerBase::SharedPtr _timer;
	std::unique_ptr<search::SearchPlanner> _planner;

	mutable std::mutex _status_mutex;
	std::optional<px4_msgs::msg::VehicleStatus> _vehicle_status;
	std::optional<bool> _landed;
	SteadyClock::time_point _vehicle_status_received_at{};
	std::chrono::system_clock::time_point _vehicle_status_received_wall{};
	std::uint64_t _last_vehicle_status_timestamp{0U};
	bool _vehicle_status_timestamp_valid{false};

	std::mutex _command_mutex;
	std::deque<std::string> _command_queue;
	std::unordered_map<std::string, std::string> _processed_responses;
	std::deque<std::string> _processed_request_order;

	std::mutex _mqtt_mutex;
	struct mosquitto * _mosquitto{nullptr};
	bool _mosquitto_initialized{false};
	bool _mqtt_loop_started{false};
	std::atomic_bool _mqtt_connected{false};
	std::atomic_bool _publish_initial_state{false};

	std::string _manual_plan_directory;
	std::string _working_plan_directory;
	std::string _default_manual_plan;
	bool _reset_working_plan{false};

	std::string _mqtt_host;
	int _mqtt_port{8883};
	std::string _mqtt_username;
	std::string _mqtt_password;
	std::string _mqtt_client_id{"search_bridge"};
	std::string _mqtt_topic_prefix{"search"};
	std::string _mqtt_tls_ca_file;
	std::string _status_topic;
	std::string _plans_topic;
	std::string _command_topic;
	std::string _response_topic;
	std::string _planner_status_topic;
	double _px4_status_timeout_s{2.0};
	int _status_publish_period_ms{500};
	int _planner_publish_period_ms{2000};

	std::uint64_t _status_sequence{0U};
	SteadyClock::time_point _last_status_publish{};
	SteadyClock::time_point _last_planner_publish{};
	std::string _last_planner_error;
};
