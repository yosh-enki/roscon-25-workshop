#include "search/SearchMode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include <cmath>
#include <stdexcept>

namespace {
constexpr char kModeName[] = "Search";
constexpr char kNodeName[] = "search";
constexpr bool kEnableDebugOutput = true;
}

SearchMode::SearchMode(rclcpp::Node & node)
: ModeBase(node, px4_ros2::ModeBase::Settings{kModeName, false})
, _node(node)
{
	_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
	_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
	_map_projection = std::make_shared<px4_ros2::MapProjection>(*this);

	loadParameters();
	_planner = std::make_unique<search::SearchPlanner>(
		_param_manual_plan_directory,
		_param_working_plan_directory,
		_param_default_manual_plan,
		_param_reset_working_plan);

	RCLCPP_INFO(
		node.get_logger(),
		"SearchMode (%s) initialized. manual_plan_directory='%s', "
		"working_plan_directory='%s', default_manual_plan='%s'",
		kModeName,
		_param_manual_plan_directory.c_str(),
		_param_working_plan_directory.c_str(),
		_param_default_manual_plan.c_str());
}

void SearchMode::loadParameters()
{
	_node.declare_parameter<std::string>("manual_plan_directory", "");
	_node.declare_parameter<std::string>("working_plan_directory", "");
	_node.declare_parameter<std::string>("default_manual_plan", "aavc2026_mission.plan");
	_node.declare_parameter<bool>("reset_working_plan", false);
	_node.declare_parameter<float>("search_altitude_m", 15.0F);
	_node.declare_parameter<float>("max_horizontal_speed_m_s", 5.0F);
	_node.declare_parameter<float>("waypoint_reach_radius_m", 4.0F);
	_node.declare_parameter<float>("max_yaw_rate_rad_s", 2.0F);

	_node.get_parameter("manual_plan_directory", _param_manual_plan_directory);
	_node.get_parameter("working_plan_directory", _param_working_plan_directory);
	_node.get_parameter("default_manual_plan", _param_default_manual_plan);
	_node.get_parameter("reset_working_plan", _param_reset_working_plan);
	_node.get_parameter("search_altitude_m", _param_search_altitude_m);
	_node.get_parameter("max_horizontal_speed_m_s", _param_max_horizontal_speed_m_s);
	_node.get_parameter("waypoint_reach_radius_m", _param_waypoint_reach_radius_m);
	_node.get_parameter("max_yaw_rate_rad_s", _param_max_yaw_rate_rad_s);

	if (!std::isfinite(_param_search_altitude_m) ||
	    !std::isfinite(_param_max_horizontal_speed_m_s) ||
	    !std::isfinite(_param_waypoint_reach_radius_m) ||
	    !std::isfinite(_param_max_yaw_rate_rad_s) ||
	    _param_max_horizontal_speed_m_s <= 0.0F ||
	    _param_waypoint_reach_radius_m <= 0.0F ||
	    _param_max_yaw_rate_rad_s <= 0.0F) {
		throw std::invalid_argument(
			"Search parameters must be finite; speed, reach radius and yaw rate must be positive");
	}

	resolvePlanPaths();

	RCLCPP_INFO(
		_node.get_logger(),
		"Search parameters: manual_plan_directory='%s', working_plan_directory='%s', "
		"default_manual_plan='%s', reset_working_plan=%s, search_altitude_m=%.1f, "
		"max_horizontal_speed_m_s=%.1f, waypoint_reach_radius_m=%.1f, max_yaw_rate_rad_s=%.1f",
		_param_manual_plan_directory.c_str(),
		_param_working_plan_directory.c_str(),
		_param_default_manual_plan.c_str(),
		_param_reset_working_plan ? "true" : "false",
		_param_search_altitude_m,
		_param_max_horizontal_speed_m_s,
		_param_waypoint_reach_radius_m,
		_param_max_yaw_rate_rad_s);
}

void SearchMode::resolvePlanPaths()
{
	std::string package_share;
	if (_param_manual_plan_directory.empty() || _param_working_plan_directory.empty()) {
		try {
			package_share = ament_index_cpp::get_package_share_directory("search");
		} catch (const std::exception & error) {
			RCLCPP_WARN(
				_node.get_logger(),
				"Could not resolve search package share directory (%s); "
				"using paths relative to the current working directory",
				error.what());
		}
	}

	if (_param_manual_plan_directory.empty()) {
		_param_manual_plan_directory = package_share.empty()
			? "plans/manual"
			: package_share + "/plans/manual";
	}
	if (_param_working_plan_directory.empty()) {
		_param_working_plan_directory = package_share.empty()
			? "plans/working"
			: package_share + "/plans/working";
	}
}

bool SearchMode::loadAndConvertWaypoints()
{
	if (!_parse_failed && _plan.waypoints.empty()) {
		try {
			_plan = _planner->loadWorkingPlan();
			_route = _planner->routeForSearch(_plan);
		} catch (const std::exception & error) {
			_parse_failed = true;
			RCLCPP_ERROR(
				_node.get_logger(),
				"Search: failed to prepare working plan '%s': %s",
				_planner->workingPlanFile().c_str(), error.what());
			return false;
		}
	}

	if (_route.waypoints.empty() || !_map_projection->isInitialized()) {
		return false;
	}

	if (_param_search_altitude_m <= 0.0F) {
		_param_search_altitude_m = _plan.altitude;
	}

	_waypoints.clear();
	_waypoints.reserve(_route.waypoints.size());
	for (const auto & waypoint : _route.waypoints) {
		const Eigen::Vector2d global{waypoint[0], waypoint[1]};
		const Eigen::Vector2f local_xy = _map_projection->globalToLocal(global);
		_waypoints.emplace_back(
			local_xy.x(), local_xy.y(), -_param_search_altitude_m);
	}

	_waypoints_loaded = true;
	_current_waypoint_index = 0U;

	RCLCPP_INFO(
		_node.get_logger(),
		"Search: loaded %zu active waypoint(s) from working plan; "
		"next original waypoint index=%zu, altitude=%.1f m",
		_waypoints.size(), _plan.next_waypoint_index, _param_search_altitude_m);
	return true;
}

void SearchMode::onActivate()
{
	_waypoints_loaded = false;
	_parse_failed = false;
	_mode_finished = false;
	_plan = search::SearchPlan{};
	_route = search::SearchPlanner::Route{};
	_waypoints.clear();
	_current_waypoint_index = 0U;

	RCLCPP_INFO(
		_node.get_logger(),
		"Search activated; SearchPlanner will load '%s'",
		_planner->workingPlanFile().c_str());
}

void SearchMode::updateWorkingPlanOnDeactivate()
{
	if (!_global_position->positionValid()) {
		RCLCPP_WARN(
			_node.get_logger(),
			"Search deactivated without a valid global position; working plan was not updated");
		return;
	}

	try {
		// The mode may be deactivated before its first setpoint update. Load the
		// plan in that case so that the current position is still persisted.
		if (_plan.waypoints.empty()) {
			_plan = _planner->loadWorkingPlan();
			_route = _planner->routeForSearch(_plan);
		}

		const Eigen::Vector3d global_position = _global_position->position();
		const std::array<double, 2> entry_point{
			global_position.x(), global_position.y()};
		_planner->updateEntryPoint(
			entry_point, _plan, _route, _current_waypoint_index);

		std::size_t next_index = _route.first_plan_waypoint_index;
		if (_route.starts_with_entry_point && _current_waypoint_index > 0U) {
			next_index += _current_waypoint_index - 1U;
		} else if (!_route.starts_with_entry_point) {
			next_index += _current_waypoint_index;
		}
		if (next_index > _plan.waypoints.size()) {
			next_index = _plan.waypoints.size();
		}

		RCLCPP_INFO(
			_node.get_logger(),
			"SearchPlanner updated working plan entry point to lat=%.7f lon=%.7f; "
			"next waypoint index=%zu",
			entry_point[0], entry_point[1], next_index);
	} catch (const std::exception & error) {
		RCLCPP_ERROR(
			_node.get_logger(),
			"SearchPlanner could not update working plan '%s': %s",
			_planner->workingPlanFile().c_str(), error.what());
	}
}

void SearchMode::onDeactivate()
{
	updateWorkingPlanOnDeactivate();
	RCLCPP_INFO(_node.get_logger(), "Search deactivated");
}

void SearchMode::updateSetpoint(float dt_s)
{
	(void)dt_s;
	if (_mode_finished) {
		return;
	}

	if (!_local_position->positionXYValid() || !_local_position->positionZValid()) {
		return;
	}
	if (!_waypoints_loaded) {
		if (!loadAndConvertWaypoints()) {
			RCLCPP_WARN_THROTTLE(
				_node.get_logger(), *_node.get_clock(), 2'000'000'000,
				"Search waiting for working plan / map projection reference to initialize...");
			return;
		}
	}
	if (_waypoints.empty()) {
		_mode_finished = true;
		completed(px4_ros2::Result::ModeFailureOther);
		return;
	}

	const Eigen::Vector3f position = _local_position->positionNed();
	const float cruise_z = -_param_search_altitude_m;
	const float reach_radius = _param_waypoint_reach_radius_m;

	// First reach the configured search altitude from the current XY position.
	if (position.z() > cruise_z + reach_radius) {
		Eigen::Vector3f climb_target = position;
		climb_target.z() = cruise_z;
		_goto_setpoint->update(climb_target);
		return;
	}

	const Eigen::Vector2f position_xy(position.x(), position.y());
	if (_current_waypoint_index < _waypoints.size()) {
		const Eigen::Vector2f waypoint_xy = _waypoints[_current_waypoint_index].head<2>();
		if ((position_xy - waypoint_xy).norm() < reach_radius) {
			RCLCPP_INFO(
				_node.get_logger(),
				"Search reached active waypoint %zu/%zu",
				_current_waypoint_index + 1U, _waypoints.size());
			++_current_waypoint_index;
		}
	}

	if (_current_waypoint_index >= _waypoints.size()) {
		_goto_setpoint->update(_waypoints.back());
		_mode_finished = true;
		RCLCPP_INFO(_node.get_logger(), "Search complete; holding over the last waypoint");
		completed(px4_ros2::Result::Success);
		return;
	}

	const Eigen::Vector3f target = _waypoints[_current_waypoint_index];
	const Eigen::Vector2f to_waypoint = target.head<2>() - position_xy;
	const float distance_to_waypoint = to_waypoint.norm();

	if (distance_to_waypoint < 0.1F) {
		_goto_setpoint->update(target);
	} else {
		// NED convention: 0 rad is North and +pi/2 rad is East.
		const float heading = std::atan2(to_waypoint.y(), to_waypoint.x());
		_goto_setpoint->update(
			target,
			heading,
			_param_max_horizontal_speed_m_s,
			std::nullopt,
			_param_max_yaw_rate_rad_s);
	}
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<SearchMode>>(
		kNodeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
