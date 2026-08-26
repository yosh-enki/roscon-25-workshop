#include "domain/working_plan.hpp"

#include <cmath>

namespace full_self_driving::domain
{

WorkingPlan::WorkingPlan(
  const std::string & working_plan_id,
  const std::string & source_artifact_id,
  const std::string & map_id,
  const std::string & scenario_id,
  const std::string & source_artifact_sha256,
  const CanonicalSearchRoute & source_route,
  const Route & transit_in_route,
  const Route & transit_out_route)
: working_plan_id_(working_plan_id)
, source_artifact_id_(source_artifact_id)
, map_id_(map_id)
, scenario_id_(scenario_id)
, source_artifact_sha256_(source_artifact_sha256)
, canonical_route_sha256_(source_route.canonical_route_sha256)
, generation_(1)
, state_(WorkingPlanState::READY)
, durability_state_(WorkingPlanDurability::SYNCED)
, source_route_(source_route)
, transit_in_route_(transit_in_route)
, transit_out_route_(transit_out_route)
{
  checkpoint_.working_plan_id = working_plan_id_;
  checkpoint_.generation = 1;
  checkpoint_.next_source_index = 0;
  checkpoint_.has_checkpoint_position = false;
  checkpoint_.checkpoint_latitude_deg = 0.0;
  checkpoint_.checkpoint_longitude_deg = 0.0;
  checkpoint_.checkpoint_altitude_m = 0.0;
  checkpoint_.completed_waypoints = 0;
  checkpoint_.total_waypoints = static_cast<uint32_t>(source_route_.waypoints.size());
  checkpoint_.progress_percent = 0.0f;
  checkpoint_.checkpoint_reason = "INITIAL";
  checkpoint_.checkpoint_sequence = 1;
}

bool WorkingPlan::reset(
  [[maybe_unused]] uint64_t expected_revision,
  const std::string & confirmation,
  std::string * out_error)
{
  if (confirmation != "CONFIRM") {
    if (out_error) {
      *out_error = "WorkingPlan reset requires confirmation token 'CONFIRM'";
    }
    return false;
  }

  // Increment generation
  generation_++;

  // Reset checkpoint state to empty and 0%
  checkpoint_.generation = generation_;
  checkpoint_.next_source_index = 0;
  checkpoint_.has_checkpoint_position = false;
  checkpoint_.checkpoint_latitude_deg = 0.0;
  checkpoint_.checkpoint_longitude_deg = 0.0;
  checkpoint_.checkpoint_altitude_m = 0.0;
  checkpoint_.completed_waypoints = 0;
  checkpoint_.progress_percent = 0.0f;
  checkpoint_.checkpoint_reason = "RESET";
  checkpoint_.checkpoint_sequence++;

  state_ = WorkingPlanState::READY;
  return true;
}

bool WorkingPlan::update_checkpoint(
  const SearchCheckpointData & cp,
  const std::string & reason,
  std::string * out_error)
{
  if (cp.completed_waypoints > cp.total_waypoints) {
    if (out_error) {
      *out_error = "Completed waypoints cannot exceed total waypoints";
    }
    return false;
  }

  checkpoint_ = cp;
  checkpoint_.working_plan_id = working_plan_id_;
  checkpoint_.generation = generation_;
  checkpoint_.checkpoint_reason = reason;
  checkpoint_.checkpoint_sequence++;

  if (checkpoint_.total_waypoints > 0) {
    checkpoint_.progress_percent = (static_cast<float>(checkpoint_.completed_waypoints) /
                                    static_cast<float>(checkpoint_.total_waypoints)) * 100.0f;
  } else {
    checkpoint_.progress_percent = 0.0f;
  }

  if (checkpoint_.completed_waypoints >= checkpoint_.total_waypoints && checkpoint_.total_waypoints > 0) {
    state_ = WorkingPlanState::COMPLETE;
  } else {
    state_ = WorkingPlanState::SEARCHING;
  }

  return true;
}

CanonicalSearchRoute WorkingPlan::route_for_search() const
{
  CanonicalSearchRoute route;
  route.default_altitude_m = source_route_.default_altitude_m;
  route.cruise_speed_m_s = source_route_.cruise_speed_m_s;

  // If checkpoint has an interrupted position, add it as first waypoint (resume point)
  if (checkpoint_.has_checkpoint_position) {
    SearchWaypoint entry_wp;
    entry_wp.latitude_deg = checkpoint_.checkpoint_latitude_deg;
    entry_wp.longitude_deg = checkpoint_.checkpoint_longitude_deg;
    entry_wp.altitude_m = checkpoint_.checkpoint_altitude_m > 0.0 ?
                          checkpoint_.checkpoint_altitude_m : source_route_.default_altitude_m;
    entry_wp.source_index = checkpoint_.next_source_index;
    route.waypoints.push_back(entry_wp);
  }

  // Add remaining unsearched waypoints from next_source_index to end
  for (size_t i = checkpoint_.next_source_index; i < source_route_.waypoints.size(); ++i) {
    route.waypoints.push_back(source_route_.waypoints[i]);
  }

  // If route is empty (e.g. all waypoints completed), hold at the last waypoint
  if (route.waypoints.empty() && !source_route_.waypoints.empty()) {
    route.waypoints.push_back(source_route_.waypoints.back());
  }

  route.canonical_route_sha256 = PlanParser::compute_canonical_route_hash(route);
  return route;
}

}  // namespace full_self_driving::domain
