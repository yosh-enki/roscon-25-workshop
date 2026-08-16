#pragma once

#include <aruco_database/msg/aruco_database_status.hpp>
#include <aruco_database/msg/aruco_detection_array.hpp>
#include <aruco_database/msg/aruco_marker_array.hpp>
#include <aruco_database/srv/clear_aruco_database.hpp>
#include <aruco_database/srv/get_aruco_position.hpp>
#include <aruco_database/srv/list_aruco_markers.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class ArucoDatabaseNode final : public rclcpp::Node
{
public:
	ArucoDatabaseNode();
	~ArucoDatabaseNode() override;

private:
	using SteadyClock = std::chrono::steady_clock;

	struct MarkerRecord
	{
		double latitude_deg{};
		double longitude_deg{};
		std::uint64_t observation_count{};
	};

	struct DatabaseView
	{
		std::uint64_t revision{};
		bool database_dirty{false};
		bool persistence_ok{true};
		std::string persistence_error;
		std::vector<std::pair<int, MarkerRecord>> markers;
	};

	void loadParameters();
	void loadDatabase();
	void saveDatabaseTimerCallback();
	bool saveDatabase(bool force = false, std::string * error_message = nullptr);
	void saveDatabaseAfterUpdate(bool immediate);
	bool createBackupFile(std::string & backup_file, std::string & error_message) const;

	void publishState();
	void publishSnapshot(const DatabaseView & view);
	void publishStatus(const DatabaseView & view);
	DatabaseView databaseView() const;

	void detectionsCallback(
		const aruco_database::msg::ArucoDetectionArray::SharedPtr msg);
	void globalPositionCallback(
		const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
	void getPositionCallback(
		const aruco_database::srv::GetArucoPosition::Request::SharedPtr request,
		aruco_database::srv::GetArucoPosition::Response::SharedPtr response);
	void listMarkersCallback(
		const aruco_database::srv::ListArucoMarkers::Request::SharedPtr request,
		aruco_database::srv::ListArucoMarkers::Response::SharedPtr response);
	void clearDatabaseCallback(
		const aruco_database::srv::ClearArucoDatabase::Request::SharedPtr request,
		aruco_database::srv::ClearArucoDatabase::Response::SharedPtr response);

	bool transformToGlobal(
		const std_msgs::msg::Header & header,
		const geometry_msgs::msg::Pose & pose,
		const geometry_msgs::msg::TransformStamped & world_transform,
		double & latitude_deg,
		double & longitude_deg) const;

	static bool isFinitePose(const geometry_msgs::msg::Pose & pose);
	static std::string expandUserPath(const std::string & path);
	static bool validLatitude(double latitude_deg);
	static bool validLongitude(double longitude_deg);
	bool isOriginReady() const;

	std::string _detection_topic;
	std::string _global_position_topic;
	std::string _vehicle_frame;
	std::string _database_file;
	std::string _world_frame;
	bool _auto_origin{true};
	bool _origin_configured{false};
	bool _origin_ready{false};
	double _origin_world_north_m{};
	double _origin_world_east_m{};
	double _origin_latitude_deg{};
	double _origin_longitude_deg{};
	double _save_period_s{};
	double _transform_timeout_s{};
	bool _save_on_update{true};
	std::int64_t _save_min_interval_ms{200};

	std::unordered_map<int, MarkerRecord> _markers;
	mutable std::mutex _markers_mutex;
	mutable std::mutex _origin_mutex;
	std::recursive_mutex _database_operation_mutex;
	bool _database_dirty{false};
	bool _persistence_ok{true};
	std::string _persistence_error;
	std::uint64_t _revision{0U};
	SteadyClock::time_point _last_save_attempt{};

	std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
	std::unique_ptr<tf2_ros::TransformListener> _tf_listener;

	rclcpp::Subscription<aruco_database::msg::ArucoDetectionArray>::SharedPtr _detections_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr _global_position_sub;
	rclcpp::Publisher<aruco_database::msg::ArucoMarkerArray>::SharedPtr _markers_pub;
	rclcpp::Publisher<aruco_database::msg::ArucoDatabaseStatus>::SharedPtr _status_pub;
	rclcpp::Service<aruco_database::srv::GetArucoPosition>::SharedPtr _get_position_service;
	rclcpp::Service<aruco_database::srv::ListArucoMarkers>::SharedPtr _list_markers_service;
	rclcpp::Service<aruco_database::srv::ClearArucoDatabase>::SharedPtr _clear_database_service;
	rclcpp::TimerBase::SharedPtr _save_timer;
};
