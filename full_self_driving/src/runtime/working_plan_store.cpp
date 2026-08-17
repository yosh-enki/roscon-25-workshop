#include "runtime/working_plan_store.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"

namespace full_self_driving::runtime
{

WorkingPlanStore::WorkingPlanStore(const std::string & working_directory)
: working_directory_(working_directory)
{
  if (!working_directory_.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(working_directory_, ec);
  }
}

bool WorkingPlanStore::save_working_plan(
  const domain::WorkingPlan & working_plan,
  std::string * out_error)
{
  if (working_directory_.empty()) {
    return true;
  }

  std::filesystem::path target =
    std::filesystem::path(working_directory_) / (working_plan.get_working_plan_id() + ".json");
  std::filesystem::path temp =
    std::filesystem::path(working_directory_) / (working_plan.get_working_plan_id() + ".tmp");

  std::optional<std::array<double, 2>> entry_pt;
  if (working_plan.get_checkpoint().has_checkpoint_position) {
    entry_pt = std::array<double, 2>{
      working_plan.get_checkpoint().checkpoint_latitude_deg,
      working_plan.get_checkpoint().checkpoint_longitude_deg
    };
  }

  std::string dumped = domain::PlanPrinter::print(
    working_plan.get_source_route(),
    entry_pt,
    working_plan.get_checkpoint().next_source_index);

  std::ofstream out(temp);
  if (!out) {
    if (out_error) {
      *out_error = "Failed to open temporary file for working plan";
    }
    return false;
  }

  out << dumped;
  out.flush();
  out.close();

  std::error_code ec;
  std::filesystem::rename(temp, target, ec);
  if (ec) {
    std::filesystem::remove(temp, ec);
    if (out_error) {
      *out_error = "Failed to rename working plan file: " + ec.message();
    }
    return false;
  }

  return true;
}

std::optional<domain::WorkingPlan> WorkingPlanStore::load_working_plan(
  const std::string & working_plan_id,
  std::string * out_error) const
{
  if (working_directory_.empty()) {
    if (out_error) {
      *out_error = "Working directory is not configured";
    }
    return std::nullopt;
  }

  std::filesystem::path target =
    std::filesystem::path(working_directory_) / (working_plan_id + ".json");

  if (!std::filesystem::is_regular_file(target)) {
    if (out_error) {
      *out_error = "Working plan file not found: " + target.string();
    }
    return std::nullopt;
  }

  auto parse_res = domain::PlanParser::parse_file(target.string());
  if (!parse_res.is_valid) {
    if (out_error) {
      *out_error = "Failed to parse working plan file: " + parse_res.error_message;
    }
    return std::nullopt;
  }

  domain::WorkingPlan wp(
    working_plan_id,
    "art_restored",
    "kmitl_airfield",
    "default_scenario",
    parse_res.raw_content_sha256,
    parse_res.route);

  if (parse_res.has_search_planner_metadata) {
    domain::SearchCheckpointData cp;
    cp.working_plan_id = working_plan_id;
    cp.next_source_index = static_cast<uint32_t>(parse_res.next_waypoint_index);
    if (parse_res.entry_point.has_value()) {
      cp.has_checkpoint_position = true;
      cp.checkpoint_latitude_deg = (*parse_res.entry_point)[0];
      cp.checkpoint_longitude_deg = (*parse_res.entry_point)[1];
    }
    cp.completed_waypoints = cp.next_source_index;
    cp.total_waypoints = static_cast<uint32_t>(parse_res.route.waypoints.size());
    wp.update_checkpoint(cp, "RESTORED_FROM_STORE");
  }

  return wp;
}

std::vector<std::string> WorkingPlanStore::list_working_plan_ids() const
{
  std::vector<std::string> ids;
  if (working_directory_.empty() || !std::filesystem::is_directory(working_directory_)) {
    return ids;
  }

  std::error_code ec;
  for (const auto & entry : std::filesystem::directory_iterator(working_directory_, ec)) {
    if (ec) break;
    if (entry.is_regular_file() && entry.path().extension() == ".json") {
      ids.push_back(entry.path().stem().string());
    }
  }
  return ids;
}

}  // namespace full_self_driving::runtime
