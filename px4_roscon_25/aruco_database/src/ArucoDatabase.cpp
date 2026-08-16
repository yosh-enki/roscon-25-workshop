#include "aruco_database/ArucoDatabase.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>
#include <unistd.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;
constexpr char kMarkersTopic[] = "/aruco_database/markers";
constexpr char kStatusTopic[] = "/aruco_database/status";

bool yamlNumberIsFinite(const YAML::Node & node)
{
	return node && node.IsScalar() && std::isfinite(node.as<double>());
}

std::uint64_t newSessionRevision()
{
	static std::atomic<std::uint64_t> sequence{0U};
	const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	const auto serial = sequence.fetch_add(1U) + 1U;
	const auto value = static_cast<std::uint64_t>(timestamp) ^ serial;
	return value == 0U ? serial : value;
}

void syncPath(const std::filesystem::path & path, const bool directory)
{
	const int flags = O_RDONLY | (directory ? O_DIRECTORY : 0);
	const int descriptor = ::open(path.c_str(), flags);
	if (descriptor < 0) {
		throw std::runtime_error(
			"could not open " + path.string() + " for sync: " + std::strerror(errno));
	}
	if (::fsync(descriptor) != 0) {
		const int error_number = errno;
		(void)::close(descriptor);
		throw std::runtime_error(
			"could not sync " + path.string() + ": " + std::strerror(error_number));
	}
	if (::close(descriptor) != 0) {
		throw std::runtime_error(
			"could not close " + path.string() + " after sync: " + std::strerror(errno));
	}
}
}  // namespace

ArucoDatabaseNode::ArucoDatabaseNode()
	: Node("aruco_database")
{
	loadParameters();

	_tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
	_tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer, this, true);

	loadDatabase();
	if (_database_dirty) {
		std::string persistence_error;
		if (!saveDatabase(true, &persistence_error)) {
			RCLCPP_ERROR(
				get_logger(), "Could not initialize the ArUco database file: %s",
				persistence_error.c_str());
		}
	}

	const auto snapshot_qos = rclcpp::QoS(1).reliable().transient_local();
	_markers_pub = create_publisher<aruco_database::msg::ArucoMarkerArray>(
		kMarkersTopic, snapshot_qos);
	_status_pub = create_publisher<aruco_database::msg::ArucoDatabaseStatus>(
		kStatusTopic, snapshot_qos);

	const auto detection_qos = rclcpp::QoS(1).best_effort();
	_detections_sub = create_subscription<aruco_database::msg::ArucoDetectionArray>(
		_detection_topic, detection_qos,
		std::bind(&ArucoDatabaseNode::detectionsCallback, this, std::placeholders::_1));

	if (_auto_origin && !_origin_configured) {
		_global_position_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
			_global_position_topic, rclcpp::SensorDataQoS(),
			std::bind(&ArucoDatabaseNode::globalPositionCallback, this, std::placeholders::_1));
	}

	_get_position_service = create_service<aruco_database::srv::GetArucoPosition>(
		"/aruco_database/get_position",
		std::bind(
			&ArucoDatabaseNode::getPositionCallback, this,
			std::placeholders::_1, std::placeholders::_2));
	_list_markers_service = create_service<aruco_database::srv::ListArucoMarkers>(
		"/aruco_database/list_markers",
		std::bind(
			&ArucoDatabaseNode::listMarkersCallback, this,
			std::placeholders::_1, std::placeholders::_2));
	_clear_database_service = create_service<aruco_database::srv::ClearArucoDatabase>(
		"/aruco_database/clear",
		std::bind(
			&ArucoDatabaseNode::clearDatabaseCallback, this,
			std::placeholders::_1, std::placeholders::_2));

	const auto save_period_ms = std::max<std::int64_t>(
		1, static_cast<std::int64_t>(_save_period_s * 1000.0));
	_save_timer = create_wall_timer(
		std::chrono::milliseconds(save_period_ms),
		std::bind(&ArucoDatabaseNode::saveDatabaseTimerCallback, this));

	RCLCPP_INFO(
		get_logger(),
		"Aruco database ready: detection_topic='%s', global_position_topic='%s', "
		"database_file='%s', world_frame='%s', vehicle_frame='%s', "
		"save_on_update=%s, save_min_interval_ms=%ld",
		_detection_topic.c_str(), _global_position_topic.c_str(), _database_file.c_str(),
		_world_frame.c_str(), _vehicle_frame.c_str(), _save_on_update ? "true" : "false",
		static_cast<long>(_save_min_interval_ms));

	if (_origin_configured) {
		RCLCPP_INFO(
			get_logger(),
			"Using manually configured WGS84 origin (%.8f, %.8f)",
			_origin_latitude_deg, _origin_longitude_deg);
	} else if (_auto_origin) {
		RCLCPP_INFO(
			get_logger(),
			"Waiting for a valid global vehicle position on '%s' to set the launch origin",
			_global_position_topic.c_str());
	} else {
		RCLCPP_ERROR(
			get_logger(),
			"No global origin is enabled; detections will not be stored. "
			"Enable auto_origin or set origin_configured with latitude/longitude.");
	}

	publishState();
}

ArucoDatabaseNode::~ArucoDatabaseNode()
{
	(void)saveDatabase();
}

void ArucoDatabaseNode::loadParameters()
{
	_detection_topic = declare_parameter<std::string>("detection_topic", "/aruco/detections");
	_global_position_topic = declare_parameter<std::string>(
		"global_position_topic", "/fmu/out/vehicle_global_position");
	_vehicle_frame = declare_parameter<std::string>("vehicle_frame", "base_link_frd");
	_database_file = expandUserPath(declare_parameter<std::string>(
		"database_file", "database/markers.yaml"));
	_world_frame = declare_parameter<std::string>("world_frame", "odom_ned");
	_auto_origin = declare_parameter<bool>("auto_origin", true);
	_origin_configured = declare_parameter<bool>("origin_configured", false);
	_origin_latitude_deg = declare_parameter<double>("origin_latitude_deg", 0.0);
	_origin_longitude_deg = declare_parameter<double>("origin_longitude_deg", 0.0);
	_save_period_s = declare_parameter<double>("save_period_s", 2.0);
	_transform_timeout_s = declare_parameter<double>("transform_timeout_s", 0.05);
	_save_on_update = declare_parameter<bool>("save_on_update", true);
	_save_min_interval_ms = declare_parameter<std::int64_t>("save_min_interval_ms", 200);

	if (_detection_topic.empty() || _global_position_topic.empty() ||
		_vehicle_frame.empty() || _world_frame.empty()) {
		throw std::invalid_argument(
			"detection_topic, global_position_topic, vehicle_frame and world_frame must not be empty");
	}

	if (_database_file.empty()) {
		throw std::invalid_argument("database_file must not be empty");
	}

	if (_database_file.back() == '/') {
		throw std::invalid_argument("database_file must be a file path, not a directory");
	}

	if (!std::isfinite(_save_period_s) || _save_period_s <= 0.0) {
		throw std::invalid_argument("save_period_s must be finite and greater than zero");
	}

	if (!std::isfinite(_transform_timeout_s) || _transform_timeout_s < 0.0) {
		throw std::invalid_argument("transform_timeout_s must be finite and non-negative");
	}

	if (_save_min_interval_ms < 0) {
		throw std::invalid_argument("save_min_interval_ms must be non-negative");
	}

	if (_origin_configured &&
		(!validLatitude(_origin_latitude_deg) || !validLongitude(_origin_longitude_deg))) {
		throw std::invalid_argument(
			"Configured origin_latitude_deg/origin_longitude_deg is outside the valid range");
	}

	_origin_world_north_m = 0.0;
	_origin_world_east_m = 0.0;
	_origin_ready = _origin_configured;
}

void ArucoDatabaseNode::loadDatabase()
{
	if (_database_file.empty() || !std::filesystem::exists(_database_file)) {
		_revision = newSessionRevision();
		_database_dirty = true;
		RCLCPP_INFO(get_logger(), "No existing ArUco database found at '%s'", _database_file.c_str());
		return;
	}

	const YAML::Node root = YAML::LoadFile(_database_file);
	const YAML::Node persisted_revision = root["revision"];
	if (persisted_revision) {
		if (!persisted_revision.IsScalar()) {
			throw std::runtime_error("database revision must be a YAML integer");
		}
		const auto loaded_revision = persisted_revision.as<std::uint64_t>();
		if (loaded_revision == 0U) {
			_revision = newSessionRevision();
			_database_dirty = true;
		} else {
			_revision = loaded_revision;
		}
	} else {
		// Legacy files receive a new opaque generation before they are rewritten.
		_revision = newSessionRevision();
		_database_dirty = true;
	}

	const YAML::Node markers = root["markers"];
	if (!markers) {
		return;
	}
	if (!markers.IsSequence()) {
		throw std::runtime_error("database markers must be a YAML sequence");
	}

	std::lock_guard<std::mutex> lock(_markers_mutex);
	for (const auto & marker : markers) {
		if (!marker["id"] || !yamlNumberIsFinite(marker["latitude_deg"]) ||
			!yamlNumberIsFinite(marker["longitude_deg"])) {
			throw std::runtime_error(
				"each database marker requires id, latitude_deg and longitude_deg");
		}

		const int id = marker["id"].as<int>();
		const double latitude_deg = marker["latitude_deg"].as<double>();
		const double longitude_deg = marker["longitude_deg"].as<double>();
		if (!validLatitude(latitude_deg) || !validLongitude(longitude_deg)) {
			throw std::runtime_error("database marker contains an invalid latitude or longitude");
		}
		if (_markers.find(id) != _markers.end()) {
			throw std::runtime_error("database contains duplicate ArUco IDs");
		}

		MarkerRecord record;
		record.latitude_deg = latitude_deg;
		record.longitude_deg = longitude_deg;
		record.observation_count = marker["observation_count"]
			? marker["observation_count"].as<std::uint64_t>()
			: 1U;
		_markers.emplace(id, record);
	}

	RCLCPP_INFO(
		get_logger(), "Loaded %zu ArUco marker(s) from '%s' (revision=%llu)",
		_markers.size(), _database_file.c_str(),
		static_cast<unsigned long long>(_revision));
}

void ArucoDatabaseNode::saveDatabaseTimerCallback()
{
	(void)saveDatabase();
	publishStatus(databaseView());
}

bool ArucoDatabaseNode::saveDatabase(const bool force, std::string * error_message)
{
	std::lock_guard<std::recursive_mutex> operation_lock(_database_operation_mutex);
	std::vector<std::pair<int, MarkerRecord>> snapshot;
	std::uint64_t revision = 0U;
	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		if (!force && !_database_dirty) {
			if (error_message != nullptr) {
				error_message->clear();
			}
			return true;
		}
		revision = _revision;
		snapshot.reserve(_markers.size());
		for (const auto & entry : _markers) {
			snapshot.push_back(entry);
		}
		_database_dirty = false;
	}

	std::sort(snapshot.begin(), snapshot.end(), [](const auto & lhs, const auto & rhs) {
		return lhs.first < rhs.first;
	});

	try {
		const std::filesystem::path database_path(_database_file);
		if (database_path.has_parent_path()) {
			std::filesystem::create_directories(database_path.parent_path());
		}

		YAML::Emitter emitter;
		emitter << YAML::BeginMap;
		emitter << YAML::Key << "revision" << YAML::Value << revision;
		emitter << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
		for (const auto & [id, record] : snapshot) {
			emitter << YAML::BeginMap;
			emitter << YAML::Key << "id" << YAML::Value << id;
			emitter << YAML::Key << "latitude_deg" << YAML::Value << record.latitude_deg;
			emitter << YAML::Key << "longitude_deg" << YAML::Value << record.longitude_deg;
			emitter << YAML::Key << "observation_count" << YAML::Value << record.observation_count;
			emitter << YAML::EndMap;
		}
		emitter << YAML::EndSeq;
		emitter << YAML::EndMap;

		const std::filesystem::path temporary_path = _database_file + ".tmp";
		{
			std::ofstream output(temporary_path, std::ios::trunc);
			if (!output) {
				throw std::runtime_error("could not open temporary database file");
			}
			output << emitter.c_str() << '\n';
			output.flush();
			if (!output) {
				throw std::runtime_error("could not write temporary database file");
			}
		}
		syncPath(temporary_path, false);
		std::filesystem::rename(temporary_path, database_path);
		const std::filesystem::path parent_path = database_path.has_parent_path()
			? database_path.parent_path()
			: std::filesystem::path(".");
		syncPath(parent_path, true);
	} catch (const std::exception & error) {
		{
			std::lock_guard<std::mutex> lock(_markers_mutex);
			_database_dirty = true;
			_persistence_ok = false;
			_persistence_error = error.what();
		}
		if (error_message != nullptr) {
			*error_message = error.what();
		}
		RCLCPP_ERROR(get_logger(), "Could not save ArUco database: %s", error.what());
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		_persistence_ok = true;
		_persistence_error.clear();
	}
	if (error_message != nullptr) {
		error_message->clear();
	}
	return true;
}

void ArucoDatabaseNode::saveDatabaseAfterUpdate(const bool immediate)
{
	if (!_save_on_update) {
		return;
	}

	const auto now = SteadyClock::now();
	const bool first_attempt = _last_save_attempt.time_since_epoch().count() == 0;
	const bool interval_elapsed = first_attempt ||
		std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_save_attempt).count() >=
		_save_min_interval_ms;
	if (!immediate && !interval_elapsed) {
		return;
	}

	_last_save_attempt = now;
	(void)saveDatabase();
	publishStatus(databaseView());
}

bool ArucoDatabaseNode::createBackupFile(
	std::string & backup_file, std::string & error_message) const
{
	backup_file.clear();
	error_message.clear();

	try {
		if (!std::filesystem::exists(_database_file)) {
			return true;
		}

		const auto now = std::chrono::system_clock::now();
		const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
			now.time_since_epoch()) % 1000;
		const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
		std::tm local_time{};
		if (const std::tm * converted = std::gmtime(&now_time); converted != nullptr) {
			local_time = *converted;
		}

		std::ostringstream suffix;
		suffix << std::put_time(&local_time, "%Y%m%dT%H%M%SZ") << '-'
			   << std::setfill('0') << std::setw(3) << milliseconds.count();
		static std::atomic<std::uint64_t> backup_sequence{0U};
		const std::uint64_t sequence = backup_sequence.fetch_add(1U);
		for (std::uint64_t attempt = 0U; attempt < 1000U; ++attempt) {
			const std::string candidate = _database_file + ".backup-" + suffix.str() + "-" +
				std::to_string(sequence + attempt);
			try {
				std::filesystem::copy_file(_database_file, candidate);
				backup_file = candidate;
				return true;
			} catch (const std::filesystem::filesystem_error &) {
				std::error_code exists_error;
				if (std::filesystem::exists(candidate, exists_error) && !exists_error) {
					continue;
				}
				throw;
			}
		}
		throw std::runtime_error("could not allocate a unique database backup path");
	} catch (const std::exception & error) {
		error_message = error.what();
		backup_file.clear();
		return false;
	}
}

ArucoDatabaseNode::DatabaseView ArucoDatabaseNode::databaseView() const
{
	DatabaseView view;
	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		view.revision = _revision;
		view.database_dirty = _database_dirty;
		view.persistence_ok = _persistence_ok;
		view.persistence_error = _persistence_error;
		view.markers.reserve(_markers.size());
		for (const auto & entry : _markers) {
			view.markers.push_back(entry);
		}
	}
	std::sort(view.markers.begin(), view.markers.end(), [](const auto & lhs, const auto & rhs) {
		return lhs.first < rhs.first;
	});
	return view;
}

void ArucoDatabaseNode::publishState()
{
	const auto view = databaseView();
	publishSnapshot(view);
	publishStatus(view);
}

void ArucoDatabaseNode::publishSnapshot(const DatabaseView & view)
{
	if (_markers_pub == nullptr) {
		return;
	}

	aruco_database::msg::ArucoMarkerArray message;
	message.header.stamp = now();
	message.revision = view.revision;
	message.markers.reserve(view.markers.size());
	for (const auto & [id, record] : view.markers) {
		aruco_database::msg::ArucoMarker marker;
		marker.id = id;
		marker.latitude_deg = record.latitude_deg;
		marker.longitude_deg = record.longitude_deg;
		marker.observation_count = record.observation_count;
		message.markers.push_back(marker);
	}
	_markers_pub->publish(message);
}

void ArucoDatabaseNode::publishStatus(const DatabaseView & view)
{
	if (_status_pub == nullptr) {
		return;
	}

	aruco_database::msg::ArucoDatabaseStatus message;
	message.header.stamp = now();
	message.revision = view.revision;
	message.marker_count = static_cast<std::uint32_t>(view.markers.size());
	message.origin_ready = isOriginReady();
	message.database_dirty = view.database_dirty;
	message.persistence_ok = view.persistence_ok;
	if (!view.persistence_ok) {
		message.persistence_state = "error";
	} else if (view.database_dirty) {
		message.persistence_state = "pending";
	} else {
		message.persistence_state = "synced";
	}
	message.last_error = view.persistence_error;
	_status_pub->publish(message);
}

void ArucoDatabaseNode::detectionsCallback(
	const aruco_database::msg::ArucoDetectionArray::SharedPtr msg)
{
	if (msg->detections.empty()) {
		return;
	}

	if (!isOriginReady()) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Ignoring ArUco detections until the WGS84 launch origin is ready");
		return;
	}

	if (msg->header.frame_id.empty()) {
		RCLCPP_WARN(get_logger(), "Ignoring ArUco detections with an empty frame_id");
		return;
	}

	geometry_msgs::msg::TransformStamped world_transform;
	try {
		world_transform = _tf_buffer->lookupTransform(
			_world_frame,
			msg->header.frame_id,
			rclcpp::Time(msg->header.stamp),
			rclcpp::Duration::from_seconds(_transform_timeout_s));
	} catch (const tf2::TransformException & error) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Could not transform ArUco detections from '%s' to '%s': %s",
			msg->header.frame_id.c_str(), _world_frame.c_str(), error.what());
		return;
	}

	bool database_changed = false;
	bool new_marker_discovered = false;
	std::lock_guard<std::recursive_mutex> operation_lock(_database_operation_mutex);
	for (const auto & detection : msg->detections) {
		if (!isFinitePose(detection.pose)) {
			RCLCPP_WARN(get_logger(), "Ignoring non-finite pose for ArUco ID %d", detection.id);
			continue;
		}

		double latitude_deg = 0.0;
		double longitude_deg = 0.0;
		if (!transformToGlobal(
				msg->header, detection.pose, world_transform, latitude_deg, longitude_deg)) {
			continue;
		}

		std::lock_guard<std::mutex> lock(_markers_mutex);
		auto [iterator, inserted] = _markers.try_emplace(detection.id);
		MarkerRecord & record = iterator->second;
		const auto next_observation_count = record.observation_count + 1U;
		if (record.observation_count == 0U) {
			record.latitude_deg = latitude_deg;
			record.longitude_deg = longitude_deg;
		} else {
			const double weight = 1.0 / static_cast<double>(next_observation_count);
			record.latitude_deg += (latitude_deg - record.latitude_deg) * weight;
			record.longitude_deg += (longitude_deg - record.longitude_deg) * weight;
		}
		record.observation_count = next_observation_count;
		_database_dirty = true;
		database_changed = true;
		new_marker_discovered = new_marker_discovered || inserted;

		if (inserted) {
			RCLCPP_INFO(get_logger(), "Discovered ArUco ID %d", detection.id);
		}
	}

	if (!database_changed) {
		return;
	}

	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		++_revision;
	}
	publishState();
	saveDatabaseAfterUpdate(new_marker_discovered);
}

void ArucoDatabaseNode::globalPositionCallback(
	const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
	if (_origin_configured || !_auto_origin || isOriginReady()) {
		return;
	}

	if (!msg->lat_lon_valid || !validLatitude(msg->lat) || !validLongitude(msg->lon)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for a valid latitude/longitude from '%s'",
			_global_position_topic.c_str());
		return;
	}

	if (msg->dead_reckoning) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Waiting for a non-dead-reckoning global position before latching the origin");
		return;
	}

	geometry_msgs::msg::TransformStamped vehicle_transform;
	try {
		// PX4 timestamps use a different clock domain, so use the latest ROS TF.
		vehicle_transform = _tf_buffer->lookupTransform(
			_world_frame,
			_vehicle_frame,
			tf2::TimePointZero,
			tf2::durationFromSec(_transform_timeout_s));
	} catch (const tf2::TransformException & error) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Could not get the vehicle pose in '%s' while setting the launch origin: %s",
			_world_frame.c_str(), error.what());
		return;
	}

	const double vehicle_north_m = vehicle_transform.transform.translation.x;
	const double vehicle_east_m = vehicle_transform.transform.translation.y;
	if (!std::isfinite(vehicle_north_m) || !std::isfinite(vehicle_east_m)) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 5000,
			"Vehicle pose is not finite; cannot set the launch origin");
		return;
	}

	{
		std::lock_guard<std::mutex> lock(_origin_mutex);
		if (_origin_ready) {
			return;
		}
		_origin_latitude_deg = msg->lat;
		_origin_longitude_deg = msg->lon;
		_origin_world_north_m = vehicle_north_m;
		_origin_world_east_m = vehicle_east_m;
		_origin_ready = true;
	}

	RCLCPP_INFO(
		get_logger(),
		"Latched launch origin from vehicle global position: "
		"lat=%.8f lon=%.8f, %s=(%.3f, %.3f) m",
		msg->lat, msg->lon, _world_frame.c_str(), vehicle_north_m, vehicle_east_m);
	publishStatus(databaseView());
}

void ArucoDatabaseNode::getPositionCallback(
	const aruco_database::srv::GetArucoPosition::Request::SharedPtr request,
	aruco_database::srv::GetArucoPosition::Response::SharedPtr response)
{
	std::lock_guard<std::mutex> lock(_markers_mutex);
	const auto iterator = _markers.find(request->aruco_id);
	if (iterator == _markers.end()) {
		response->found = false;
		response->error_message =
			"ArUco ID " + std::to_string(request->aruco_id) + " is not in the database";
		return;
	}

	response->found = true;
	response->latitude_deg = iterator->second.latitude_deg;
	response->longitude_deg = iterator->second.longitude_deg;
	response->error_message.clear();
}

void ArucoDatabaseNode::listMarkersCallback(
	const aruco_database::srv::ListArucoMarkers::Request::SharedPtr request,
	aruco_database::srv::ListArucoMarkers::Response::SharedPtr response)
{
	(void)request;
	const auto view = databaseView();
	response->success = true;
	response->error_message.clear();
	response->revision = view.revision;
	response->origin_ready = isOriginReady();
	response->database_dirty = view.database_dirty;
	response->markers.clear();
	response->markers.reserve(view.markers.size());
	for (const auto & [id, record] : view.markers) {
		aruco_database::msg::ArucoMarker marker;
		marker.id = id;
		marker.latitude_deg = record.latitude_deg;
		marker.longitude_deg = record.longitude_deg;
		marker.observation_count = record.observation_count;
		response->markers.push_back(marker);
	}
}

void ArucoDatabaseNode::clearDatabaseCallback(
	const aruco_database::srv::ClearArucoDatabase::Request::SharedPtr request,
	aruco_database::srv::ClearArucoDatabase::Response::SharedPtr response)
{
	response->success = false;
	response->persisted = false;
	response->backup_file.clear();

	if (!request->confirm) {
		response->error_message = "Clear rejected: confirm must be true";
		return;
	}

	std::lock_guard<std::recursive_mutex> operation_lock(_database_operation_mutex);
	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		if (request->use_expected_revision && _revision != request->expected_revision) {
			response->error_message = "Clear rejected: database revision has changed";
			response->revision = _revision;
			response->marker_count = static_cast<std::uint32_t>(_markers.size());
			return;
		}
	}

	if (request->backup) {
		std::string sync_error;
		if (!saveDatabase(true, &sync_error)) {
			const auto view = databaseView();
			response->revision = view.revision;
			response->marker_count = static_cast<std::uint32_t>(view.markers.size());
			response->error_message =
				"Clear rejected because the current database could not be persisted: " + sync_error;
			return;
		}

		std::string backup_error;
		if (!createBackupFile(response->backup_file, backup_error)) {
			response->error_message = "Could not create database backup: " + backup_error;
			return;
		}
	}

	{
		std::lock_guard<std::mutex> lock(_markers_mutex);
		if (request->use_expected_revision && _revision != request->expected_revision) {
			response->error_message = "Clear rejected: database revision has changed";
			response->revision = _revision;
			response->marker_count = static_cast<std::uint32_t>(_markers.size());
			return;
		}
		_markers.clear();
		++_revision;
		_database_dirty = true;
	}

	std::string save_error;
	const bool persisted = saveDatabase(true, &save_error);
	const auto view = databaseView();
	response->revision = view.revision;
	response->marker_count = static_cast<std::uint32_t>(view.markers.size());
	response->persisted = persisted;
	response->success = persisted;
	if (!persisted) {
		response->error_message = "Database was cleared in memory but could not be persisted: " + save_error;
	} else {
		response->error_message.clear();
	}

	publishState();
}

bool ArucoDatabaseNode::transformToGlobal(
	const std_msgs::msg::Header & header,
	const geometry_msgs::msg::Pose & pose,
	const geometry_msgs::msg::TransformStamped & world_transform,
	double & latitude_deg,
	double & longitude_deg) const
{
	geometry_msgs::msg::PoseStamped source_pose;
	source_pose.header = header;
	source_pose.pose = pose;

	geometry_msgs::msg::PoseStamped world_pose;
	tf2::doTransform(source_pose, world_pose, world_transform);

	double origin_latitude_deg = 0.0;
	double origin_longitude_deg = 0.0;
	double origin_world_north_m = 0.0;
	double origin_world_east_m = 0.0;
	{
		std::lock_guard<std::mutex> lock(_origin_mutex);
		if (!_origin_ready) {
			return false;
		}
		origin_latitude_deg = _origin_latitude_deg;
		origin_longitude_deg = _origin_longitude_deg;
		origin_world_north_m = _origin_world_north_m;
		origin_world_east_m = _origin_world_east_m;
	}

	const double north_m = world_pose.pose.position.x - origin_world_north_m;
	const double east_m = world_pose.pose.position.y - origin_world_east_m;
	if (!std::isfinite(north_m) || !std::isfinite(east_m)) {
		return false;
	}

	const double origin_latitude_rad = origin_latitude_deg * kPi / 180.0;
	const double sin_latitude = std::sin(origin_latitude_rad);
	const double sin_squared = sin_latitude * sin_latitude;
	const double prime_vertical_radius = kWgs84SemiMajorAxisM /
		std::sqrt(1.0 - kWgs84EccentricitySquared * sin_squared);
	const double meridian_radius = kWgs84SemiMajorAxisM *
		(1.0 - kWgs84EccentricitySquared) /
		std::pow(1.0 - kWgs84EccentricitySquared * sin_squared, 1.5);
	const double cosine_latitude = std::cos(origin_latitude_rad);
	if (std::abs(cosine_latitude) < std::numeric_limits<double>::epsilon()) {
		RCLCPP_ERROR(get_logger(), "Cannot project local east displacement at the configured pole");
		return false;
	}

	latitude_deg = origin_latitude_deg + (north_m / meridian_radius) * 180.0 / kPi;
	longitude_deg = origin_longitude_deg +
		(east_m / (prime_vertical_radius * cosine_latitude)) * 180.0 / kPi;
	return validLatitude(latitude_deg) && validLongitude(longitude_deg);
}

bool ArucoDatabaseNode::isOriginReady() const
{
	std::lock_guard<std::mutex> lock(_origin_mutex);
	return _origin_ready;
}

bool ArucoDatabaseNode::isFinitePose(const geometry_msgs::msg::Pose & pose)
{
	return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
		std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
		std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
		std::isfinite(pose.orientation.w) &&
		(std::abs(pose.orientation.x) + std::abs(pose.orientation.y) +
			std::abs(pose.orientation.z) + std::abs(pose.orientation.w) > 0.0);
}

std::string ArucoDatabaseNode::expandUserPath(const std::string & path)
{
	std::string expanded_path = path;
	if (expanded_path == "~" || expanded_path.rfind("~/", 0) == 0) {
		const char * home = std::getenv("HOME");
		if (home != nullptr) {
			expanded_path = std::string(home) + expanded_path.substr(1);
		}
	}

	if (!expanded_path.empty() && std::filesystem::path(expanded_path).is_relative()) {
		try {
			return (
				std::filesystem::path(
					ament_index_cpp::get_package_share_directory("aruco_database")) /
				expanded_path).string();
		} catch (const std::exception &) {
			// Keep the relative path if the package share is unavailable.
		}
	}

	return expanded_path;
}

bool ArucoDatabaseNode::validLatitude(const double latitude_deg)
{
	return std::isfinite(latitude_deg) && latitude_deg >= -90.0 && latitude_deg <= 90.0;
}

bool ArucoDatabaseNode::validLongitude(const double longitude_deg)
{
	return std::isfinite(longitude_deg) && longitude_deg >= -180.0 && longitude_deg <= 180.0;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ArucoDatabaseNode>());
	rclcpp::shutdown();
	return 0;
}
