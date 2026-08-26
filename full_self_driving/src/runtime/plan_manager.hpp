#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "domain/mission_context.hpp"
#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"
#include "domain/working_plan.hpp"

namespace full_self_driving::runtime
{

struct ManagedPlanArtifact
{
  std::string artifact_id;
  std::string safe_name;
  std::string map_name;
  std::string sha256;
  uint64_t byte_length{0};
  bool immutable{true};
  std::vector<uint8_t> raw_content;
  domain::CanonicalSearchRoute route;
  std::vector<domain::RoutePoint> transit_in_waypoints;
  std::vector<domain::RoutePoint> transit_out_waypoints;
};

class PlanManager
{
public:
  explicit PlanManager(const std::string & storage_directory = "");
  ~PlanManager() = default;

  const std::string & get_storage_directory() const { return storage_directory_; }

  std::optional<ManagedPlanArtifact> upload_artifact(
    const std::string & safe_name,
    const std::vector<uint8_t> & bytes,
    uint64_t expected_revision = 0,
    std::string * out_error = nullptr);

  std::vector<ManagedPlanArtifact> list_artifacts() const;

  std::optional<ManagedPlanArtifact> get_artifact(const std::string & artifact_id) const;

  std::optional<domain::WorkingPlan> create_or_select_working_plan(
    const std::string & artifact_id,
    const std::string & map_id,
    const std::string & scenario_id,
    uint64_t expected_revision = 0,
    std::string * out_error = nullptr);

  std::optional<domain::WorkingPlan> reset_working_plan(
    const std::string & working_plan_id,
    uint64_t expected_revision,
    const std::string & confirmation,
    std::string * out_error = nullptr);

  std::optional<domain::WorkingPlan> update_checkpoint(
    const std::string & working_plan_id,
    const domain::SearchCheckpointData & checkpoint,
    const std::string & reason,
    std::string * out_error = nullptr);

  std::optional<domain::WorkingPlan> get_working_plan(const std::string & working_plan_id) const;

  std::optional<domain::CanonicalSearchRoute> route_for_search(const std::string & working_plan_id) const;

  std::optional<domain::WorkingPlan> get_active_working_plan(
    const std::string & map_id,
    const std::string & scenario_id) const;

private:
  std::string storage_directory_;
  mutable std::mutex mutex_;
  std::map<std::string, ManagedPlanArtifact> artifacts_;
  std::map<std::string, domain::WorkingPlan> working_plans_;
  std::map<std::string, std::string> active_working_plans_by_scope_;
};

}  // namespace full_self_driving::runtime
