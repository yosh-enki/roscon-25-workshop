#include "domain/mission_context.hpp"

#include <algorithm>
#include <iomanip>
#include <openssl/evp.h>
#include <sstream>

namespace full_self_driving::domain
{

MissionContext::MissionContext(const std::string & context_id)
: context_id_(context_id)
{
}

void MissionContext::set_armed(bool armed)
{
  armed_ = armed;
}

bool MissionContext::set_engineering_config(
  std::shared_ptr<const EngineeringConfig> config,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot change configuration while locked or armed";
    return false;
  }

  if (!config) {
    if (out_error) *out_error = "Config pointer cannot be null";
    return false;
  }

  auto validation = config->validate();
  if (!validation.is_valid) {
    state_ = ConfigState::CONFIG_INVALID;
    if (out_error) {
      std::ostringstream ss;
      for (size_t i = 0; i < validation.violations.size(); ++i) {
        if (i > 0) ss << "; ";
        ss << validation.violations[i];
      }
      *out_error = ss.str();
    }
    return false;
  }

  resolved_config_ = config;
  resolved_config_hash_ = config->compute_canonical_hash();
  selection_.resolved_config_hash = resolved_config_hash_;

  if (state_ == ConfigState::CONFIG_INVALID || state_ == ConfigState::STARTUP) {
    state_ = ConfigState::STANDBY;
  }
  return true;
}

bool MissionContext::edit_selection(
  const OperatorSelection & new_sel,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot edit selection while locked or armed";
    return false;
  }

  if (state_ == ConfigState::RECOVERY_REQUIRED) {
    if (out_error) *out_error = "Cannot edit selection during recovery";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  selection_ = new_sel;
  selection_.selection_revision = expected_revision + 1;
  if (!resolved_config_hash_.empty()) {
    selection_.resolved_config_hash = resolved_config_hash_;
  }

  if (state_ == ConfigState::STANDBY || state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
  return true;
}

bool MissionContext::select_map_scenario(
  const std::string & map_id,
  const std::string & scenario_id,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot edit selection while locked or armed";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  if (map_id.empty() || scenario_id.empty()) {
    if (out_error) *out_error = "Map ID and Scenario ID must not be empty";
    return false;
  }

  selection_.map_id = map_id;
  selection_.scenario_id = scenario_id;
  selection_.selection_revision = expected_revision + 1;

  if (state_ == ConfigState::STANDBY || state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
  return true;
}

bool MissionContext::select_target(
  const TargetIdentity & target,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot edit selection while locked or armed";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  if (!target.is_valid()) {
    if (out_error) *out_error = "Target identity is invalid";
    return false;
  }

  if (resolved_config_) {
    const auto & tc = resolved_config_->target_constraints;
    if (target.marker_id < tc.marker_id_min || target.marker_id > tc.marker_id_max) {
      if (out_error) *out_error = "Target marker ID is outside allowed range";
      return false;
    }

    auto dict_it = std::find(tc.allowed_dictionaries.begin(), tc.allowed_dictionaries.end(), target.dictionary);
    if (dict_it == tc.allowed_dictionaries.end()) {
      if (out_error) *out_error = "Target dictionary '" + target.dictionary + "' is not allowed";
      return false;
    }

    auto ns_it = std::find(tc.allowed_namespaces.begin(), tc.allowed_namespaces.end(), target.target_namespace);
    if (ns_it == tc.allowed_namespaces.end()) {
      if (out_error) *out_error = "Target namespace '" + target.target_namespace + "' is not allowed";
      return false;
    }
  }

  selection_.target = target;
  selection_.selection_revision = expected_revision + 1;

  if (state_ == ConfigState::STANDBY || state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
  return true;
}

void MissionContext::clear_target()
{
  selection_.target.reset();
  selection_.selection_revision++;
  if (state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
}

bool MissionContext::select_plan_artifact(
  const std::string & artifact_id,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot edit selection while locked or armed";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  if (artifact_id.empty()) {
    if (out_error) *out_error = "Plan artifact ID must not be empty";
    return false;
  }

  selection_.plan_artifact_id = artifact_id;
  selection_.selection_revision = expected_revision + 1;

  if (state_ == ConfigState::STANDBY || state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
  return true;
}

bool MissionContext::select_working_plan(
  const std::string & working_plan_id,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot edit selection while locked or armed";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  if (working_plan_id.empty()) {
    if (out_error) *out_error = "Working plan ID must not be empty";
    return false;
  }

  selection_.working_plan_id = working_plan_id;
  selection_.selection_revision = expected_revision + 1;

  if (state_ == ConfigState::STANDBY || state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
  return true;
}

ValidationReport MissionContext::validate_selection(uint64_t expected_revision)
{
  ValidationReport report;

  if (locked_ || armed_) {
    report.add_violation("Cannot validate while locked or armed");
    return report;
  }

  if (expected_revision != selection_.selection_revision) {
    report.add_violation("Selection revision mismatch");
    return report;
  }

  if (selection_.map_id.empty()) {
    report.add_violation("Map ID is required");
  }

  if (selection_.scenario_id.empty()) {
    report.add_violation("Scenario ID is required");
  }

  if (!selection_.target.has_value()) {
    report.add_violation("Target identity is required");
  } else if (!selection_.target->is_valid()) {
    report.add_violation("Target identity is invalid");
  }

  if (!resolved_config_) {
    report.add_violation("Authoritative engineering config is not set");
  }

  if (report.violations.empty()) {
    report.is_valid = true;
    std::ostringstream ss;
    ss << context_id_ << ":" << selection_.selection_revision << ":" << resolved_config_hash_;
    std::string token_src = ss.str();

    unsigned char hash[EVP_MAX_MD_SIZE];
    size_t hash_len = 0;
    EVP_Q_digest(nullptr, "SHA256", nullptr, token_src.data(), token_src.size(), hash, &hash_len);

    std::ostringstream hex_stream;
    hex_stream << std::hex << std::setfill('0');
    for (size_t i = 0; i < std::min(static_cast<size_t>(16), hash_len); ++i) {
      hex_stream << std::setw(2) << static_cast<int>(hash[i]);
    }
    validation_token_ = "tok_" + hex_stream.str();
    report.token = validation_token_;
    state_ = ConfigState::VALIDATING;
  } else {
    report.is_valid = false;
    validation_token_.clear();
  }

  return report;
}

bool MissionContext::commit(
  const std::string & validation_token,
  uint64_t expected_revision,
  std::string * out_error)
{
  if (locked_ || armed_) {
    if (out_error) *out_error = "Cannot commit while locked or armed";
    return false;
  }

  if (expected_revision != selection_.selection_revision) {
    if (out_error) {
      *out_error = "Selection revision mismatch: expected " +
                   std::to_string(expected_revision) + ", actual " +
                   std::to_string(selection_.selection_revision);
    }
    return false;
  }

  if (validation_token.empty() || validation_token != validation_token_) {
    if (out_error) *out_error = "Invalid or expired validation token";
    return false;
  }

  committed_revision_ = selection_.selection_revision;
  resolved_config_hash_ = selection_.resolved_config_hash;
  state_ = ConfigState::COMMITTED;
  validation_token_.clear();
  return true;
}

bool MissionContext::check_readiness(
  bool px4_transport_ok,
  bool storage_ok,
  bool health_ok,
  std::vector<std::string> * out_missing_gates)
{
  std::vector<std::string> missing;

  if (state_ != ConfigState::COMMITTED && state_ != ConfigState::READY_FOR_OWNMODE) {
    missing.push_back("Mission context is not committed");
  }

  if (selection_.map_id.empty() || selection_.scenario_id.empty()) {
    missing.push_back("Map and scenario scope must be set");
  }

  if (!selection_.target.has_value() || !selection_.target->is_valid()) {
    missing.push_back("Target identity must be committed and valid");
  }

  if (!resolved_config_ || resolved_config_hash_.empty()) {
    missing.push_back("Authoritative engineering config must be loaded and hashed");
  }

  if (!px4_transport_ok) {
    missing.push_back("PX4 transport is not ready");
  }

  if (!storage_ok) {
    missing.push_back("Durable storage is not healthy");
  }

  if (!health_ok) {
    missing.push_back("Component health check failed");
  }

  if (out_missing_gates) {
    *out_missing_gates = missing;
  }

  if (missing.empty()) {
    state_ = ConfigState::READY_FOR_OWNMODE;
    return true;
  }

  if (state_ == ConfigState::READY_FOR_OWNMODE) {
    state_ = ConfigState::COMMITTED;
  }
  return false;
}

bool MissionContext::lock(
  const std::string & mission_id,
  const std::string & sortie_id,
  std::string * out_error)
{
  if (locked_) {
    if (out_error) *out_error = "Mission context is already locked";
    return false;
  }

  if (state_ != ConfigState::READY_FOR_OWNMODE && state_ != ConfigState::COMMITTED) {
    if (out_error) *out_error = "Cannot lock uncommitted or unready context";
    return false;
  }

  if (mission_id.empty() || sortie_id.empty()) {
    if (out_error) *out_error = "Mission ID and Sortie ID must not be empty";
    return false;
  }

  locked_ = true;
  mission_id_ = mission_id;
  sortie_id_ = sortie_id;
  state_ = ConfigState::LOCKED;
  return true;
}

bool MissionContext::unlock(
  bool is_disarmed,
  std::string * out_error)
{
  if (!is_disarmed || armed_) {
    if (out_error) *out_error = "Vehicle must be disarmed to unlock mission context";
    return false;
  }

  locked_ = false;
  mission_id_.clear();
  sortie_id_.clear();
  state_ = ConfigState::STANDBY;
  return true;
}

}  // namespace full_self_driving::domain
