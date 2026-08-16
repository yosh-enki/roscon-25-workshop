#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geodesic.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "search/PlanParser.hpp"
#include "search/SearchPlanner.hpp"

/**
 * @brief Search custom flight mode for a QGroundControl search plan.
 *
 * Search obtains its active coordinates from SearchPlanner. SearchPlanner
 * keeps immutable manual plans in plans/manual, creates timestamped copies in
 * plans/working, and stores the latest entry point and next unvisited waypoint
 * in the active copy when this mode is deactivated.
 */
class SearchMode final : public px4_ros2::ModeBase
{
public:
	explicit SearchMode(rclcpp::Node & node);

	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	void loadParameters();
	void resolvePlanPaths();
	bool loadAndConvertWaypoints();
	void updateWorkingPlanOnDeactivate();

	rclcpp::Node & _node;

	std::shared_ptr<px4_ros2::OdometryGlobalPosition> _global_position;
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_position;
	std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
	std::shared_ptr<px4_ros2::MapProjection> _map_projection;
	std::unique_ptr<search::SearchPlanner> _planner;

	search::SearchPlan _plan;
	search::SearchPlanner::Route _route;
	std::vector<Eigen::Vector3f> _waypoints;
	std::size_t _current_waypoint_index{0U};
	bool _waypoints_loaded{false};
	bool _parse_failed{false};
	bool _mode_finished{false};

	// Plan lifecycle parameters.
	std::string _param_manual_plan_directory;
	std::string _param_working_plan_directory;
	std::string _param_default_manual_plan;
	bool _param_reset_working_plan{false};

	// Flight parameters.
	float _param_search_altitude_m{15.0F};
	float _param_max_horizontal_speed_m_s{5.0F};
	float _param_waypoint_reach_radius_m{4.0F};
	float _param_max_yaw_rate_rad_s{2.0F};
};
