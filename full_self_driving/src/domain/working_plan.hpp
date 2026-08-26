#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "domain/plan_parser.hpp"

namespace full_self_driving::domain
{

enum class WorkingPlanState : uint8_t
{
  MISSING = 0,
  READY = 1,
  SEARCHING = 2,
  COMPLETE = 3,
  INVALID = 4,
  RECOVERY_REQUIRED = 5
};

enum class WorkingPlanDurability : uint8_t
{
  UNKNOWN = 0,
  SYNCED = 1,
  DIRTY = 2,
  FAILED = 3
};

struct SearchCheckpointData
{
  std::string working_plan_id;
  uint64_t generation{1};
  uint32_t next_source_index{0};
  bool has_checkpoint_position{false};
  double checkpoint_latitude_deg{0.0};
  double checkpoint_longitude_deg{0.0};
  double checkpoint_altitude_m{0.0};
  uint32_t completed_waypoints{0};
  uint32_t total_waypoints{0};
  float progress_percent{0.0f};
  std::string checkpoint_reason{"INITIAL"};
  uint64_t checkpoint_sequence{1};
  uint64_t updated_monotonic_ns{0};
};

class WorkingPlan
{
public:
  WorkingPlan() = default;
  WorkingPlan(
    const std::string & working_plan_id,
    const std::string & source_artifact_id,
    const std::string & map_id,
    const std::string & scenario_id,
    const std::string & source_artifact_sha256,
    const CanonicalSearchRoute & source_route,
    const Route & transit_in_route = Route(),
    const Route & transit_out_route = Route());

  const std::string & get_working_plan_id() const { return working_plan_id_; }
  const std::string & get_source_artifact_id() const { return source_artifact_id_; }
  const std::string & get_map_id() const { return map_id_; }
  const std::string & get_scenario_id() const { return scenario_id_; }
  const std::string & get_source_artifact_sha256() const { return source_artifact_sha256_; }
  const std::string & get_canonical_route_sha256() const { return canonical_route_sha256_; }
  uint64_t get_generation() const { return generation_; }

  WorkingPlanState get_state() const { return state_; }
  void set_state(WorkingPlanState s) { state_ = s; }

  WorkingPlanDurability get_durability_state() const { return durability_state_; }
  void set_durability_state(WorkingPlanDurability d) { durability_state_ = d; }

  const CanonicalSearchRoute & get_source_route() const { return source_route_; }
  const SearchCheckpointData & get_checkpoint() const { return checkpoint_; }

  bool has_transit_in_route() const { return !transit_in_route_.empty(); }
  bool has_transit_out_route() const { return !transit_out_route_.empty(); }
  const Route & get_transit_in_route() const { return transit_in_route_; }
  const Route & get_transit_out_route() const { return transit_out_route_; }
  void set_transit_in_route(const Route & route) { transit_in_route_ = route; }
  void set_transit_out_route(const Route & route) { transit_out_route_ = route; }

  bool reset(
    uint64_t expected_revision,
    const std::string & confirmation,
    std::string * out_error = nullptr);

  bool update_checkpoint(
    const SearchCheckpointData & checkpoint,
    const std::string & reason,
    std::string * out_error = nullptr);

  CanonicalSearchRoute route_for_search() const;

private:
  std::string working_plan_id_;
  std::string source_artifact_id_;
  std::string map_id_{"kmitl_airfield"};
  std::string scenario_id_{"default_scenario"};
  std::string source_artifact_sha256_;
  std::string canonical_route_sha256_;
  uint64_t generation_{1};
  WorkingPlanState state_{WorkingPlanState::READY};
  WorkingPlanDurability durability_state_{WorkingPlanDurability::SYNCED};
  CanonicalSearchRoute source_route_;
  Route transit_in_route_;
  Route transit_out_route_;
  SearchCheckpointData checkpoint_;
};

}  // namespace full_self_driving::domain
