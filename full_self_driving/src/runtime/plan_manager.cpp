#include "runtime/plan_manager.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace full_self_driving::runtime
{

PlanManager::PlanManager(const std::string & storage_directory)
: storage_directory_(storage_directory)
{
  if (!storage_directory_.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(
      std::filesystem::path(storage_directory_) / "artifacts", ec);
    std::filesystem::create_directories(
      std::filesystem::path(storage_directory_) / "working", ec);
  }
}

std::optional<ManagedPlanArtifact> PlanManager::upload_artifact(
  const std::string & safe_name,
  const std::vector<uint8_t> & bytes,
  [[maybe_unused]] uint64_t expected_revision,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!domain::PlanParser::is_safe_basename(safe_name)) {
    if (out_error) {
      *out_error = "Invalid artifact safe_name: must be a safe .plan basename without path traversal";
    }
    return std::nullopt;
  }

  auto parse_result = domain::PlanParser::parse_bytes(bytes, safe_name);
  if (!parse_result.is_valid) {
    if (out_error) {
      *out_error = "Failed to parse plan artifact: " + parse_result.error_message;
    }
    return std::nullopt;
  }

  std::string sha256 = parse_result.raw_content_sha256;
  std::string artifact_id = "art_" + sha256.substr(0, 16);

  // Check if artifact ID already exists (Requirement 2.11 / Property 4)
  auto it = artifacts_.find(artifact_id);
  if (it != artifacts_.end()) {
    if (it->second.sha256 == sha256) {
      // Idempotent success
      return it->second;
    } else {
      // Reject hash-changing replacement!
      if (out_error) {
        *out_error = "Artifact ID collision with differing content hash: replacement rejected";
      }
      return std::nullopt;
    }
  }

  ManagedPlanArtifact artifact;
  artifact.artifact_id = artifact_id;
  artifact.safe_name = safe_name;
  artifact.sha256 = sha256;
  artifact.byte_length = bytes.size();
  artifact.immutable = true;
  artifact.raw_content = bytes;
  artifact.route = std::move(parse_result.route);

  // If storage directory is configured, write atomically to disk
  if (!storage_directory_.empty()) {
    std::filesystem::path target_path =
      std::filesystem::path(storage_directory_) / "artifacts" / (artifact_id + ".plan");
    std::filesystem::path temp_path =
      std::filesystem::path(storage_directory_) / "artifacts" / (artifact_id + ".tmp");

    std::ofstream out(temp_path, std::ios::binary);
    if (!out) {
      if (out_error) {
        *out_error = "Cannot open temporary artifact file for writing";
      }
      return std::nullopt;
    }
    out.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    out.flush();
    out.close();

    std::error_code ec;
    std::filesystem::rename(temp_path, target_path, ec);
    if (ec) {
      std::filesystem::remove(temp_path, ec);
      if (out_error) {
        *out_error = "Cannot store artifact file: " + ec.message();
      }
      return std::nullopt;
    }
  }

  artifacts_[artifact_id] = artifact;
  return artifact;
}

std::vector<ManagedPlanArtifact> PlanManager::list_artifacts() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ManagedPlanArtifact> list;
  list.reserve(artifacts_.size());
  for (const auto & [_, art] : artifacts_) {
    list.push_back(art);
  }
  return list;
}

std::optional<ManagedPlanArtifact> PlanManager::get_artifact(const std::string & artifact_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = artifacts_.find(artifact_id);
  if (it == artifacts_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<domain::WorkingPlan> PlanManager::create_or_select_working_plan(
  const std::string & artifact_id,
  const std::string & map_id,
  const std::string & scenario_id,
  [[maybe_unused]] uint64_t expected_revision,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = artifacts_.find(artifact_id);
  if (it == artifacts_.end()) {
    if (out_error) {
      *out_error = "Artifact not found: " + artifact_id;
    }
    return std::nullopt;
  }

  const auto & artifact = it->second;
  std::string working_plan_id = "wp_" + artifact_id + "_" + map_id + "_" + scenario_id;

  domain::WorkingPlan wp(
    working_plan_id,
    artifact_id,
    map_id,
    scenario_id,
    artifact.sha256,
    artifact.route);

  working_plans_[working_plan_id] = wp;
  active_working_plans_by_scope_[map_id + ":" + scenario_id] = working_plan_id;

  return wp;
}

std::optional<domain::WorkingPlan> PlanManager::reset_working_plan(
  const std::string & working_plan_id,
  uint64_t expected_revision,
  const std::string & confirmation,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = working_plans_.find(working_plan_id);
  if (it == working_plans_.end()) {
    if (out_error) {
      *out_error = "Working plan not found: " + working_plan_id;
    }
    return std::nullopt;
  }

  if (!it->second.reset(expected_revision, confirmation, out_error)) {
    return std::nullopt;
  }

  return it->second;
}

std::optional<domain::WorkingPlan> PlanManager::update_checkpoint(
  const std::string & working_plan_id,
  const domain::SearchCheckpointData & checkpoint,
  const std::string & reason,
  std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto it = working_plans_.find(working_plan_id);
  if (it == working_plans_.end()) {
    if (out_error) {
      *out_error = "Working plan not found: " + working_plan_id;
    }
    return std::nullopt;
  }

  if (!it->second.update_checkpoint(checkpoint, reason, out_error)) {
    return std::nullopt;
  }

  return it->second;
}

std::optional<domain::WorkingPlan> PlanManager::get_working_plan(const std::string & working_plan_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = working_plans_.find(working_plan_id);
  if (it == working_plans_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<domain::CanonicalSearchRoute> PlanManager::route_for_search(const std::string & working_plan_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = working_plans_.find(working_plan_id);
  if (it == working_plans_.end()) {
    return std::nullopt;
  }
  return it->second.route_for_search();
}

std::optional<domain::WorkingPlan> PlanManager::get_active_working_plan(
  const std::string & map_id,
  const std::string & scenario_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::string scope_key = map_id + ":" + scenario_id;
  auto it_id = active_working_plans_by_scope_.find(scope_key);
  if (it_id == active_working_plans_by_scope_.end()) {
    return std::nullopt;
  }

  auto it_wp = working_plans_.find(it_id->second);
  if (it_wp == working_plans_.end()) {
    return std::nullopt;
  }
  return it_wp->second;
}

}  // namespace full_self_driving::runtime
