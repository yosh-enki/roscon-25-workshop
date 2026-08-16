#pragma once

#include "search/PlanParser.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace search {

/**
 * @brief Owns the manual/working plan lifecycle and Search resume metadata.
 *
 * SearchPlanner deliberately works with directories instead of one permanent
 * working filename. Every reset creates a new UTC timestamped copy, while the
 * newest valid copy is selected when Search is activated. This keeps previous
 * working plans available for inspection and lets a bridge reset the plan
 * without exposing arbitrary filesystem paths.
 */
class SearchPlanner
{
public:
	struct Route
	{
		std::vector<std::array<double, 2>> waypoints;
		std::size_t first_plan_waypoint_index{0U};
		bool starts_with_entry_point{false};
	};

	SearchPlanner(
		std::string manual_plan_directory,
		std::string working_plan_directory,
		std::string default_manual_plan,
		bool reset_working_plan = false);

	/**
	 * @brief Validate the directories and select/create the active working plan.
	 *
	 * Initialization is lazy so the planner can also be used by the MQTT bridge
	 * for list/reset commands without creating a working copy at construction.
	 */
	void initialize();

	/**
	 * @brief Load the newest valid timestamped working plan.
	 *
	 * The directory is rescanned on every load. A reset performed by the
	 * search_bridge while Search is inactive is therefore picked up on the next
	 * activation instead of being hidden behind a cached path.
	 */
	SearchPlan loadWorkingPlan();

	Route routeForSearch(const SearchPlan & plan) const;

	/**
	 * @brief Save the current global position and remaining-plan index.
	 *
	 * route_waypoint_index is the index currently being flown in the route
	 * returned by routeForSearch(). The current waypoint is retained so the
	 * next activation resumes from the point where Search was interrupted.
	 */
	void updateEntryPoint(
		const std::array<double, 2> & current_position,
		const SearchPlan & plan,
		const Route & route,
		std::size_t route_waypoint_index);

	/// Return safe .plan basenames available in the manual-plan directory.
	std::vector<std::string> listManualPlanNames() const;

	/**
	 * @brief Create a new timestamped working copy from a manual-plan basename.
	 *
	 * Only a basename from manual_plan_directory is accepted. The destination
	 * is never overwritten, even if two reset requests arrive in one second.
	 * The returned value is the new working-plan basename.
	 */
	std::string resetWorkingPlan(const std::string & manual_plan_name);

	/**
	 * @brief Refresh the active selection from the shared working directory.
	 *
	 * This is useful to a second process, such as search_bridge, because Search
	 * and the bridge intentionally own separate SearchPlanner instances.
	 */
	void refreshActiveWorkingPlan();

	const std::string & manualPlanDirectory() const { return _manual_plan_directory; }
	const std::string & workingPlanDirectory() const { return _working_plan_directory; }
	const std::string & defaultManualPlan() const { return _default_manual_plan; }
	const std::string & workingPlanFile() const { return _active_working_plan_file; }

	std::string activeWorkingPlanName() const;

private:
	static bool isSafePlanBasename(const std::string & name);
	static bool isTimestampedWorkingPlanName(const std::string & name);

	void validateConfiguration() const;
	void ensureWorkingDirectory() const;
	std::filesystem::path manualPlanPath(const std::string & manual_plan_name) const;
	void validateManualPlan(const std::filesystem::path & manual_path) const;
	std::optional<std::filesystem::path> findLatestValidWorkingPlan() const;
	std::string createWorkingCopy(const std::string & manual_plan_name);
	std::filesystem::path activeMarkerPath() const;
	void writeActiveWorkingPlan(const std::filesystem::path & working_plan) const;

	std::string _manual_plan_directory;
	std::string _working_plan_directory;
	std::string _default_manual_plan;
	std::string _active_working_plan_file;
	bool _reset_working_plan{false};
	bool _initialized{false};
};

}  // namespace search
