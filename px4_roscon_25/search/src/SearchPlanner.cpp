#include "search/SearchPlanner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace search {

namespace {

std::string utcTimestamp(std::chrono::system_clock::time_point time)
{
	const std::time_t time_value = std::chrono::system_clock::to_time_t(time);
	std::tm utc_time{};
	if (gmtime_r(&time_value, &utc_time) == nullptr) {
		throw std::runtime_error("SearchPlanner: cannot convert current time to UTC");
	}

	std::ostringstream timestamp;
	timestamp << std::put_time(&utc_time, "%Y%m%dT%H%M%SZ");
	return timestamp.str();
}

}  // namespace

SearchPlanner::SearchPlanner(
	std::string manual_plan_directory,
	std::string working_plan_directory,
	std::string default_manual_plan,
	bool reset_working_plan)
: _manual_plan_directory(std::move(manual_plan_directory))
, _working_plan_directory(std::move(working_plan_directory))
, _default_manual_plan(std::move(default_manual_plan))
, _reset_working_plan(reset_working_plan)
{
}

bool SearchPlanner::isSafePlanBasename(const std::string & name)
{
	if (name.empty() || name == "." || name == ".." ||
		name.find('/') != std::string::npos || name.find('\\') != std::string::npos) {
		return false;
	}

	const std::filesystem::path path(name);
	return path.filename().string() == name && path.extension() == ".plan";
}

bool SearchPlanner::isTimestampedWorkingPlanName(const std::string & name)
{
	// YYYYMMDDTHHMMSSZ_<manual basename>.plan
	if (name.size() < 18U || name[8] != 'T' || name[15] != 'Z' || name[16] != '_') {
		return false;
	}
	for (std::size_t index = 0U; index < 8U; ++index) {
		if (name[index] < '0' || name[index] > '9') {
			return false;
		}
	}
	for (std::size_t index = 9U; index < 15U; ++index) {
		if (name[index] < '0' || name[index] > '9') {
			return false;
		}
	}
	return isSafePlanBasename(name.substr(17U));
}

void SearchPlanner::validateConfiguration() const
{
	if (_manual_plan_directory.empty() || _working_plan_directory.empty()) {
		throw std::runtime_error(
			"SearchPlanner: manual_plan_directory and working_plan_directory must not be empty");
	}
	if (_default_manual_plan.empty() || !isSafePlanBasename(_default_manual_plan)) {
		throw std::runtime_error(
			"SearchPlanner: default_manual_plan must be a .plan basename without path separators");
	}

	std::error_code absolute_error;
	const auto manual_directory = std::filesystem::absolute(
		std::filesystem::path(_manual_plan_directory), absolute_error).lexically_normal();
	if (absolute_error) {
		throw std::runtime_error(
			"SearchPlanner: cannot resolve manual_plan_directory: " + absolute_error.message());
	}
	const auto working_directory = std::filesystem::absolute(
		std::filesystem::path(_working_plan_directory), absolute_error).lexically_normal();
	if (absolute_error) {
		throw std::runtime_error(
			"SearchPlanner: cannot resolve working_plan_directory: " + absolute_error.message());
	}
	if (manual_directory == working_directory) {
		throw std::runtime_error(
			"SearchPlanner: manual_plan_directory and working_plan_directory must be different");
	}
}

void SearchPlanner::ensureWorkingDirectory() const
{
	const std::filesystem::path directory(_working_plan_directory);
	std::error_code error;
	std::filesystem::create_directories(directory, error);
	if (error) {
		throw std::runtime_error(
			"SearchPlanner: cannot create working-plan directory '" +
			directory.string() + "': " + error.message());
	}
	if (!std::filesystem::is_directory(directory)) {
		throw std::runtime_error(
			"SearchPlanner: working_plan_directory is not a directory: " + directory.string());
	}
}

std::filesystem::path SearchPlanner::manualPlanPath(const std::string & manual_plan_name) const
{
	if (!isSafePlanBasename(manual_plan_name)) {
		throw std::invalid_argument(
			"SearchPlanner: manual plan name must be a .plan basename without path separators");
	}
	return std::filesystem::path(_manual_plan_directory) / manual_plan_name;
}

void SearchPlanner::validateManualPlan(const std::filesystem::path & manual_path) const
{
	if (!std::filesystem::is_regular_file(manual_path)) {
		throw std::runtime_error(
			"SearchPlanner: manual plan does not exist or is not a regular file: " +
			manual_path.string());
	}

	// Validate before copying so a reset cannot make an unusable plan the active
	// working plan. The parser also verifies that the file contains waypoints.
	const SearchPlan source_plan = parseSearchPlan(manual_path.string());
	if (source_plan.has_search_planner_metadata) {
		throw std::runtime_error(
			"SearchPlanner: manual plan must not contain mission.searchPlanner resume metadata: " +
			manual_path.filename().string());
	}
}

std::filesystem::path SearchPlanner::activeMarkerPath() const
{
	return std::filesystem::path(_working_plan_directory) / ".active_working_plan";
}

void SearchPlanner::writeActiveWorkingPlan(const std::filesystem::path & working_plan) const
{
	const std::string name = working_plan.filename().string();
	if (!isTimestampedWorkingPlanName(name)) {
		throw std::runtime_error(
			"SearchPlanner: cannot mark a non-timestamped working plan as active");
	}

	const std::filesystem::path marker = activeMarkerPath();
	const std::filesystem::path temporary_marker = marker.string() + ".tmp";
	{
		std::ofstream output(temporary_marker, std::ios::trunc);
		if (!output) {
			throw std::runtime_error(
			"SearchPlanner: cannot write active working-plan marker: " + marker.string());
		}
		output << name << '\n';
		if (!output) {
			throw std::runtime_error(
			"SearchPlanner: failed while writing active working-plan marker: " +
				marker.string());
		}
	}

	std::error_code error;
	std::filesystem::rename(temporary_marker, marker, error);
	if (error) {
		std::filesystem::remove(temporary_marker);
		throw std::runtime_error(
			"SearchPlanner: cannot replace active working-plan marker: " + error.message());
	}
}

std::optional<std::filesystem::path> SearchPlanner::findLatestValidWorkingPlan() const
{
	const std::filesystem::path directory(_working_plan_directory);
	if (!std::filesystem::is_directory(directory)) {
		return std::nullopt;
	}

	// The marker records reset order across different source basenames. The
	// filename timestamp remains human-readable, while the marker prevents two
	// same-second resets from being ordered by basename accidentally.
	{
		std::ifstream marker_input(activeMarkerPath());
		std::string marked_name;
		if (marker_input >> marked_name && isTimestampedWorkingPlanName(marked_name)) {
			const std::filesystem::path marked_plan = directory / marked_name;
			if (std::filesystem::is_regular_file(marked_plan)) {
				try {
					(void)parseSearchPlan(marked_plan.string());
					return marked_plan;
				} catch (const std::exception &) {
					// Fall through to the historical-file scan below.
				}
			}
		}
	}

	std::optional<std::filesystem::path> latest;
	std::error_code iteration_error;
	for (const auto & entry : std::filesystem::directory_iterator(directory, iteration_error)) {
		if (iteration_error) {
			break;
		}
		if (!entry.is_regular_file() || entry.path().extension() != ".plan") {
			continue;
		}

		const std::string name = entry.path().filename().string();
		if (!isTimestampedWorkingPlanName(name)) {
			continue;
		}

		try {
			(void)parseSearchPlan(entry.path().string());
	} catch (const std::exception &) {
			// Invalid historical files must not prevent a valid newer plan from
			// being selected. A reset can create a fresh valid copy instead.
			continue;
		}

		if (!latest.has_value() || name > latest->filename().string()) {
			latest = entry.path();
		}
	}
	if (iteration_error) {
		throw std::runtime_error(
			"SearchPlanner: cannot inspect working-plan directory '" +
			directory.string() + "': " + iteration_error.message());
	}
	return latest;
}

std::string SearchPlanner::createWorkingCopy(const std::string & manual_plan_name)
{
	const std::filesystem::path source = manualPlanPath(manual_plan_name);
	validateManualPlan(source);
	ensureWorkingDirectory();

	const auto now = std::chrono::system_clock::now();
	for (unsigned int attempt = 0U; attempt < 100000U; ++attempt) {
		const auto candidate_time = now + std::chrono::seconds(attempt);
		const std::string candidate_name = utcTimestamp(candidate_time) + "_" + manual_plan_name;
		const std::filesystem::path destination =
			std::filesystem::path(_working_plan_directory) / candidate_name;

		std::error_code copy_error;
		std::filesystem::copy_file(
			source, destination, std::filesystem::copy_options::none, copy_error);
		if (!copy_error) {
			try {
				writeActiveWorkingPlan(destination);
			} catch (...) {
				std::error_code remove_error;
				std::filesystem::remove(destination, remove_error);
				throw;
			}
			return destination.string();
		}
		if (copy_error == std::errc::file_exists) {
			continue;
		}
		throw std::runtime_error(
			"SearchPlanner: cannot copy manual plan to working plan: " +
			copy_error.message());
	}

	throw std::runtime_error(
		"SearchPlanner: could not create a unique timestamped working-plan filename");
}

void SearchPlanner::initialize()
{
	if (_initialized) {
		return;
	}

	validateConfiguration();
	ensureWorkingDirectory();

	if (_reset_working_plan) {
		_active_working_plan_file = createWorkingCopy(_default_manual_plan);
	} else if (const auto latest = findLatestValidWorkingPlan(); latest.has_value()) {
		_active_working_plan_file = latest->string();
	} else {
		_active_working_plan_file = createWorkingCopy(_default_manual_plan);
	}

	_initialized = true;
}

SearchPlan SearchPlanner::loadWorkingPlan()
{
	initialize();

	// A reset command can create a newer copy while Search is inactive. Prefer
	// it, but keep the current path if no valid timestamped file is available.
	if (const auto latest = findLatestValidWorkingPlan(); latest.has_value()) {
		_active_working_plan_file = latest->string();
	}
	return parseSearchPlan(_active_working_plan_file);
}

SearchPlanner::Route SearchPlanner::routeForSearch(const SearchPlan & plan) const
{
	if (plan.waypoints.empty()) {
		throw std::runtime_error("SearchPlanner: cannot create a route without waypoints");
	}
	if (plan.next_waypoint_index > plan.waypoints.size()) {
		throw std::runtime_error(
			"SearchPlanner: next waypoint index exceeds the working plan");
	}

	Route route;
	route.first_plan_waypoint_index = plan.next_waypoint_index;
	if (plan.entry_point.has_value()) {
		route.starts_with_entry_point = true;
		route.waypoints.push_back(*plan.entry_point);
	}

	for (std::size_t index = plan.next_waypoint_index;
		 index < plan.waypoints.size(); ++index) {
		route.waypoints.push_back(plan.waypoints[index]);
	}

	// A completed working plan still has a useful point to hold at. This also
	// keeps SearchMode from dereferencing an empty route after a late deactivate.
	if (route.waypoints.empty()) {
		route.waypoints.push_back(plan.waypoints.back());
	}

	return route;
}

void SearchPlanner::updateEntryPoint(
	const std::array<double, 2> & current_position,
	const SearchPlan & plan,
	const Route & route,
	std::size_t route_waypoint_index)
{
	initialize();
	if (!std::isfinite(current_position[0]) || !std::isfinite(current_position[1]) ||
		current_position[0] < -90.0 || current_position[0] > 90.0 ||
		current_position[1] < -180.0 || current_position[1] > 180.0) {
		throw std::runtime_error("SearchPlanner: current global position is invalid");
	}
	if (route.waypoints.empty()) {
		throw std::runtime_error("SearchPlanner: cannot update an empty route");
	}
	if (route_waypoint_index > route.waypoints.size()) {
		throw std::runtime_error("SearchPlanner: route waypoint index is out of range");
	}

	std::size_t next_plan_waypoint_index = route.first_plan_waypoint_index;
	if (route.starts_with_entry_point) {
		if (route_waypoint_index > 0U) {
			next_plan_waypoint_index += route_waypoint_index - 1U;
		}
	} else {
		next_plan_waypoint_index += route_waypoint_index;
	}
	if (next_plan_waypoint_index > plan.waypoints.size()) {
		next_plan_waypoint_index = plan.waypoints.size();
	}

	updateSearchPlanEntryPoint(
		_active_working_plan_file, current_position, next_plan_waypoint_index);
}

std::vector<std::string> SearchPlanner::listManualPlanNames() const
{
	std::vector<std::string> names;
	const std::filesystem::path directory(_manual_plan_directory);
	if (!std::filesystem::is_directory(directory)) {
		return names;
	}

	std::error_code iteration_error;
	for (const auto & entry : std::filesystem::directory_iterator(directory, iteration_error)) {
		if (iteration_error) {
			break;
		}
		if (entry.is_regular_file() && entry.path().extension() == ".plan") {
			const std::string name = entry.path().filename().string();
			if (isSafePlanBasename(name)) {
				try {
					validateManualPlan(entry.path());
					names.push_back(name);
				} catch (const std::exception &) {
					// Keep malformed or resume-bearing files out of the
					// Node-RED selection list.
				}
			}
		}
	}
	if (iteration_error) {
		throw std::runtime_error(
			"SearchPlanner: cannot inspect manual-plan directory '" +
			directory.string() + "': " + iteration_error.message());
	}

	std::sort(names.begin(), names.end());
	return names;
}

std::string SearchPlanner::resetWorkingPlan(const std::string & manual_plan_name)
{
	validateConfiguration();
	const std::string working_file = createWorkingCopy(manual_plan_name);
	_active_working_plan_file = working_file;
	_initialized = true;
	return activeWorkingPlanName();
}

void SearchPlanner::refreshActiveWorkingPlan()
{
	initialize();
	if (const auto latest = findLatestValidWorkingPlan(); latest.has_value()) {
		_active_working_plan_file = latest->string();
	}
}

std::string SearchPlanner::activeWorkingPlanName() const
{
	if (_active_working_plan_file.empty()) {
		return {};
	}
	return std::filesystem::path(_active_working_plan_file).filename().string();
}

}  // namespace search
