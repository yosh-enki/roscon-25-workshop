#include "gateway/fsd_gateway.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

namespace full_self_driving::gateway
{

const std::set<std::string> FsdGateway::ALLOWED_COMMANDS = {
  "select_map_scenario",
  "select_target_identity",
  "select_plan_artifact",
  "create_or_select_working_plan",
  "reset_working_plan",
  "upload_plan_artifact",
  "prepare_payload",
  "clear_pad_registry",
  "validate_mission_context",
  "commit_mission_context",
  "resolve_recovery",
  "list_plan_artifacts",
  "inspect_pad_registry",
  "inspect_recovery",
  "get_status",
  "get_evidence_manifest"
};

const std::set<std::string> FsdGateway::FORBIDDEN_COMMANDS = {
  "arm",
  "disarm",
  "ownmode",
  "takeoff",
  "land",
  "rtl",
  "goto",
  "setpoint",
  "raw_control",
  "release",
  "execute_mission",
  "kill",
  "override",
  "direct_actuator",
  "emergency_drop",
  "shell",
  "reboot",
  "shutdown",
  "set_parameter",
  "publish_raw"
};

FsdGateway::FsdGateway(
  const GatewaySecurityPolicy & policy,
  std::shared_ptr<domain::MissionContext> context,
  std::shared_ptr<runtime::PlanManager> plan_manager,
  std::shared_ptr<registry::PadRegistry> registry,
  std::shared_ptr<persistence::PersistenceManager> persistence)
: policy_(policy),
  context_(context),
  plan_manager_(plan_manager),
  registry_(registry),
  persistence_(persistence)
{
}

void FsdGateway::set_policy(const GatewaySecurityPolicy & policy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  policy_ = policy;
}

void FsdGateway::set_mission_context(std::shared_ptr<domain::MissionContext> ctx)
{
  std::lock_guard<std::mutex> lock(mutex_);
  context_ = ctx;
}

void FsdGateway::set_plan_manager(std::shared_ptr<runtime::PlanManager> pm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  plan_manager_ = pm;
}

void FsdGateway::set_pad_registry(std::shared_ptr<registry::PadRegistry> reg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registry_ = reg;
}

void FsdGateway::set_persistence_manager(std::shared_ptr<persistence::PersistenceManager> pm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  persistence_ = pm;
}

void FsdGateway::set_payload_controller(std::shared_ptr<payload::PayloadController> pc)
{
  std::lock_guard<std::mutex> lock(mutex_);
  payload_controller_ = pc;
}

void FsdGateway::reset_idempotency_cache()
{
  std::lock_guard<std::mutex> lock(mutex_);
  idempotency_cache_.clear();
  forbidden_command_attempts_ = 0;
  security_violations_count_ = 0;
}

bool FsdGateway::is_command_allowed(const std::string & command) const
{
  return ALLOWED_COMMANDS.find(command) != ALLOWED_COMMANDS.end();
}

bool FsdGateway::is_command_forbidden(const std::string & command) const
{
  return FORBIDDEN_COMMANDS.find(command) != FORBIDDEN_COMMANDS.end();
}

GatewayResponse FsdGateway::process_command_json(
  const std::string & json_envelope,
  bool is_retained,
  uint64_t current_unix_ms)
{
  if (json_envelope.size() > policy_.max_payload_bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    security_violations_count_++;
    GatewayResponse resp;
    resp.accepted = false;
    resp.error_code = "ERROR_PAYLOAD_TOO_LARGE";
    resp.error_message = "Envelope size exceeds maximum allowed bytes";
    resp.severity = 2; // ERROR
    return resp;
  }

  // Parse simple JSON fields
  auto extract_str = [&](const std::string & key) -> std::string {
    auto pos = json_envelope.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = json_envelope.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = json_envelope.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = json_envelope.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return json_envelope.substr(q1 + 1, q2 - q1 - 1);
  };

  auto extract_num = [&](const std::string & key) -> uint64_t {
    auto pos = json_envelope.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    auto colon = json_envelope.find(':', pos);
    if (colon == std::string::npos) return 0;
    std::string tail = json_envelope.substr(colon + 1);
    try {
      return std::stoull(tail);
    } catch (...) {
      return 0;
    }
  };

  CommandEnvelope env;
  env.schema = extract_str("schema");
  env.request_id = extract_str("request_id");
  env.command = extract_str("command");
  env.sent_at_unix_ms = extract_num("sent_at_unix_ms");
  env.expected_revision = extract_num("expected_revision");
  env.is_retained = is_retained;
  env.raw_payload_json = json_envelope;

  return process_envelope(env, current_unix_ms);
}

GatewayResponse FsdGateway::process_envelope(
  const CommandEnvelope & env,
  uint64_t current_unix_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);

  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = false;

  // 1. Check retained command
  if (env.is_retained) {
    security_violations_count_++;
    resp.error_code = "ERROR_RETAINED_COMMAND_FORBIDDEN";
    resp.error_message = "MQTT retained commands are strictly forbidden";
    resp.severity = 2;
    return resp;
  }

  // 2. Check schema
  if (env.schema != "full_self_driving.command.v1") {
    security_violations_count_++;
    resp.error_code = "ERROR_INVALID_SCHEMA";
    resp.error_message = "Schema must be 'full_self_driving.command.v1'";
    resp.severity = 2;
    return resp;
  }

  // 3. Check request ID
  if (env.request_id.empty() || env.request_id.size() > 64) {
    security_violations_count_++;
    resp.error_code = "ERROR_INVALID_REQUEST_ID";
    resp.error_message = "Request ID must be non-empty and <= 64 characters";
    resp.severity = 2;
    return resp;
  }

  // 4. Check request age & clock skew
  if (current_unix_ms > 0 && env.sent_at_unix_ms > 0) {
    if (current_unix_ms >= env.sent_at_unix_ms) {
      double age_s = (current_unix_ms - env.sent_at_unix_ms) / 1000.0;
      if (age_s > policy_.max_request_age_s) {
        security_violations_count_++;
        resp.error_code = "ERROR_STALE_REQUEST";
        resp.error_message = "Command timestamp exceeds maximum request age";
        resp.severity = 2;
        return resp;
      }
    } else {
      double skew_s = (env.sent_at_unix_ms - current_unix_ms) / 1000.0;
      if (skew_s > policy_.max_request_age_s) {
        security_violations_count_++;
        resp.error_code = "ERROR_CLOCK_SKEW";
        resp.error_message = "Command timestamp is in the future beyond clock skew tolerance";
        resp.severity = 2;
        return resp;
      }
    }
  }

  // 5. Check forbidden command
  if (is_command_forbidden(env.command)) {
    forbidden_command_attempts_++;
    security_violations_count_++;
    resp.error_code = "ERROR_FORBIDDEN_COMMAND";
    resp.error_message = "Command '" + env.command + "' is forbidden by security boundary";
    resp.severity = 3; // FATAL/HIGH
    return resp;
  }

  // 6. Check allowlisted command
  if (!is_command_allowed(env.command)) {
    security_violations_count_++;
    resp.error_code = "ERROR_UNKNOWN_COMMAND";
    resp.error_message = "Command '" + env.command + "' is not allowlisted";
    resp.severity = 2;
    return resp;
  }

  // 7. Check idempotency cache
  auto it = idempotency_cache_.find(env.request_id);
  if (it != idempotency_cache_.end()) {
    return it->second;
  }

  // 8. Dispatch to command handler
  if (env.command == "select_map_scenario") {
    resp = handle_select_map_scenario(env);
  } else if (env.command == "select_target_identity") {
    resp = handle_select_target_identity(env);
  } else if (env.command == "select_plan_artifact") {
    resp = handle_select_plan_artifact(env);
  } else if (env.command == "create_or_select_working_plan") {
    resp = handle_create_or_select_working_plan(env);
  } else if (env.command == "reset_working_plan") {
    resp = handle_reset_working_plan(env);
  } else if (env.command == "upload_plan_artifact") {
    resp = handle_upload_plan_artifact(env);
  } else if (env.command == "prepare_payload") {
    resp = handle_prepare_payload(env);
  } else if (env.command == "clear_pad_registry") {
    resp = handle_clear_pad_registry(env);
  } else if (env.command == "validate_mission_context") {
    resp = handle_validate_mission_context(env);
  } else if (env.command == "commit_mission_context") {
    resp = handle_commit_mission_context(env);
  } else if (env.command == "resolve_recovery") {
    resp = handle_resolve_recovery(env);
  } else if (env.command == "list_plan_artifacts") {
    resp = handle_list_plan_artifacts(env);
  } else if (env.command == "inspect_pad_registry") {
    resp = handle_inspect_pad_registry(env);
  } else if (env.command == "inspect_recovery") {
    resp = handle_inspect_recovery(env);
  } else if (env.command == "get_status") {
    resp = handle_get_status(env);
  } else if (env.command == "get_evidence_manifest") {
    resp = handle_get_evidence_manifest(env);
  }

  idempotency_cache_[env.request_id] = resp;
  return resp;
}

GatewayResponse FsdGateway::handle_select_map_scenario(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_CONTEXT_UNAVAILABLE";
    resp.error_message = "MissionContext is not available";
    return resp;
  }

  // Parse map_id and scenario_id from JSON
  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string map_id = extract("map_id");
  std::string scenario_id = extract("scenario_id");

  std::string err;
  if (!context_->select_map_scenario(map_id, scenario_id, env.expected_revision, &err)) {
    resp.accepted = false;
    resp.error_code = "ERROR_SELECTION_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = context_->get_selection_revision();
  resp.response_payload_json = "{\"map_id\":\"" + map_id + "\",\"scenario_id\":\"" + scenario_id + "\"}";
  return resp;
}

GatewayResponse FsdGateway::handle_select_target_identity(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_CONTEXT_UNAVAILABLE";
    resp.error_message = "MissionContext is not available";
    return resp;
  }

  auto extract_str = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  auto extract_num = [&](const std::string & key) -> uint32_t {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return 0;
    std::string tail = env.raw_payload_json.substr(colon + 1);
    try {
      return std::stoul(tail);
    } catch (...) {
      return 0;
    }
  };

  domain::TargetIdentity target;
  target.marker_id = extract_num("marker_id");
  target.dictionary = extract_str("dictionary");
  target.target_namespace = extract_str("target_namespace");

  std::string err;
  if (!context_->select_target(target, env.expected_revision, &err)) {
    resp.accepted = false;
    resp.error_code = "ERROR_TARGET_SELECTION_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = context_->get_selection_revision();
  resp.response_payload_json = "{\"target_selected\":true}";
  return resp;
}

GatewayResponse FsdGateway::handle_select_plan_artifact(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_CONTEXT_UNAVAILABLE";
    resp.error_message = "MissionContext is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string artifact_id = extract("artifact_id");
  std::string err;
  if (!context_->select_plan_artifact(artifact_id, env.expected_revision, &err)) {
    resp.accepted = false;
    resp.error_code = "ERROR_PLAN_SELECTION_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = context_->get_selection_revision();
  resp.response_payload_json = "{\"artifact_id\":\"" + artifact_id + "\"}";
  return resp;
}

GatewayResponse FsdGateway::handle_create_or_select_working_plan(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!plan_manager_ || !context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_MANAGER_UNAVAILABLE";
    resp.error_message = "PlanManager or MissionContext is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string artifact_id = extract("artifact_id");
  std::string map_id = extract("map_id");
  std::string scenario_id = extract("scenario_id");

  std::string err;
  auto wp = plan_manager_->create_or_select_working_plan(
    artifact_id, map_id, scenario_id, env.expected_revision, &err);
  if (!wp.has_value()) {
    resp.accepted = false;
    resp.error_code = "ERROR_WORKING_PLAN_REJECTED";
    resp.error_message = err;
    return resp;
  }

  context_->select_working_plan(wp->get_working_plan_id(), env.expected_revision, &err);
  resp.accepted = true;
  resp.resulting_revision = context_->get_selection_revision();
  resp.response_payload_json = "{\"working_plan_id\":\"" + wp->get_working_plan_id() + "\"}";
  return resp;
}

GatewayResponse FsdGateway::handle_reset_working_plan(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!plan_manager_) {
    resp.accepted = false;
    resp.error_code = "ERROR_PLAN_MANAGER_UNAVAILABLE";
    resp.error_message = "PlanManager is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string wp_id = extract("working_plan_id");
  std::string conf = extract("confirmation");

  std::string err;
  auto res = plan_manager_->reset_working_plan(wp_id, env.expected_revision, conf, &err);
  if (!res.has_value()) {
    resp.accepted = false;
    resp.error_code = "ERROR_RESET_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = res->get_generation();
  resp.response_payload_json = "{\"new_generation\":" + std::to_string(res->get_generation()) + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_upload_plan_artifact(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!plan_manager_) {
    resp.accepted = false;
    resp.error_code = "ERROR_PLAN_MANAGER_UNAVAILABLE";
    resp.error_message = "PlanManager is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string safe_name = extract("safe_name");
  std::string content_str = extract("content");
  std::vector<uint8_t> bytes(content_str.begin(), content_str.end());

  std::string err;
  auto art = plan_manager_->upload_artifact(safe_name, bytes, env.expected_revision, &err);
  if (!art.has_value()) {
    resp.accepted = false;
    resp.error_code = "ERROR_UPLOAD_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.response_payload_json = "{\"artifact_id\":\"" + art->artifact_id + "\",\"sha256\":\"" + art->sha256 + "\"}";
  return resp;
}

GatewayResponse FsdGateway::handle_prepare_payload(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (context_ && (context_->is_armed() || context_->is_locked())) {
    resp.accepted = false;
    resp.error_code = "ERROR_DISARMED_REQUIRED";
    resp.error_message = "Payload preparation requires disarmed and unlocked context";
    return resp;
  }

  if (payload_controller_) {
    uint8_t op = full_self_driving::srv::PreparePayload::Request::OP_PREPARE_FOR_SORTIE;
    if (env.raw_payload_json.find("\"operation\":0") != std::string::npos ||
        env.raw_payload_json.find("OPEN_FOR_LOADING") != std::string::npos) {
      op = full_self_driving::srv::PreparePayload::Request::OP_OPEN_FOR_LOADING;
    } else if (env.raw_payload_json.find("\"operation\":1") != std::string::npos ||
               env.raw_payload_json.find("VERIFY_SECURED") != std::string::npos) {
      op = full_self_driving::srv::PreparePayload::Request::OP_VERIFY_SECURED;
    }

    full_self_driving::msg::PayloadStatus status;
    std::string err;
    bool ok = payload_controller_->prepare(op, env.request_id, env.expected_revision, status, &err);
    if (!ok) {
      resp.accepted = false;
      resp.error_code = "ERROR_PAYLOAD_PREPARATION_FAILED";
      resp.error_message = err.empty() ? "Payload preparation rejected by controller" : err;
      return resp;
    }

    resp.accepted = true;
    resp.response_payload_json = "{\"payload_prepared\":true,\"commanded_state\":" +
      std::to_string(status.commanded_state) + ",\"feedback_state\":" +
      std::to_string(status.feedback_state) + ",\"cargo_loaded\":" +
      (status.cargo_loaded ? "true" : "false") + ",\"secured\":" +
      (status.secured ? "true" : "false") + "}";
    return resp;
  }

  resp.accepted = true;
  resp.response_payload_json = "{\"payload_prepared\":true}";
  return resp;
}

GatewayResponse FsdGateway::handle_clear_pad_registry(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!registry_) {
    resp.accepted = false;
    resp.error_code = "ERROR_REGISTRY_UNAVAILABLE";
    resp.error_message = "PadRegistry is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string map_id = extract("map_id");
  std::string scenario_id = extract("scenario_id");
  std::string conf = extract("confirmation");

  bool disarmed = (context_ ? !context_->is_armed() : true);
  auto clear_res = registry_->clear(map_id, scenario_id, env.expected_revision, conf, disarmed);
  if (!clear_res.success) {
    resp.accepted = false;
    resp.error_code = "ERROR_CLEAR_REGISTRY_REJECTED";
    resp.error_message = clear_res.message;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = clear_res.committed_revision;
  resp.response_payload_json = "{\"cleared\":true,\"new_revision\":" + std::to_string(clear_res.committed_revision) + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_validate_mission_context(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_CONTEXT_UNAVAILABLE";
    resp.error_message = "MissionContext is not available";
    return resp;
  }

  auto report = context_->validate_selection(env.expected_revision);
  if (!report.is_valid) {
    resp.accepted = false;
    resp.error_code = "ERROR_VALIDATION_FAILED";
    std::ostringstream ss;
    for (size_t i = 0; i < report.violations.size(); ++i) {
      if (i > 0) ss << "; ";
      ss << report.violations[i];
    }
    resp.error_message = ss.str();
    return resp;
  }

  resp.accepted = true;
  resp.response_payload_json = "{\"validation_token\":\"" + report.token + "\"}";
  return resp;
}

GatewayResponse FsdGateway::handle_commit_mission_context(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!context_) {
    resp.accepted = false;
    resp.error_code = "ERROR_CONTEXT_UNAVAILABLE";
    resp.error_message = "MissionContext is not available";
    return resp;
  }

  auto extract = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  std::string token = extract("validation_token");
  std::string err;
  if (!context_->commit(token, env.expected_revision, &err)) {
    resp.accepted = false;
    resp.error_code = "ERROR_COMMIT_REJECTED";
    resp.error_message = err;
    return resp;
  }

  // Commit snapshot through persistence if available
  if (persistence_) {
    persistence::MissionSnapshotRecord snap;
    snap.schema_version = "1.0.0";
    snap.snapshot_revision = context_->get_committed_revision();
    snap.resolved_config_hash = context_->get_resolved_config_hash();
    snap.map_id = context_->get_selection().map_id;
    snap.scenario_id = context_->get_selection().scenario_id;
    snap.plan_artifact_id = context_->get_selection().plan_artifact_id;
    snap.working_plan_id = context_->get_selection().working_plan_id;
    if (context_->get_selection().target.has_value()) {
      snap.target = *context_->get_selection().target;
    }
    snap.checksum = snap.compute_checksum();
    persistence_->commit_snapshot(snap);
  }

  resp.accepted = true;
  resp.resulting_revision = context_->get_committed_revision();
  resp.response_payload_json = "{\"committed\":true,\"committed_revision\":" + std::to_string(context_->get_committed_revision()) + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_resolve_recovery(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;

  if (!persistence_) {
    resp.accepted = false;
    resp.error_code = "ERROR_PERSISTENCE_UNAVAILABLE";
    resp.error_message = "PersistenceManager is not available";
    return resp;
  }

  auto extract_str = [&](const std::string & key) -> std::string {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return "";
    auto q1 = env.raw_payload_json.find('"', colon);
    if (q1 == std::string::npos) return "";
    auto q2 = env.raw_payload_json.find('"', q1 + 1);
    if (q2 == std::string::npos) return "";
    return env.raw_payload_json.substr(q1 + 1, q2 - q1 - 1);
  };

  auto extract_num = [&](const std::string & key) -> uint8_t {
    auto pos = env.raw_payload_json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    auto colon = env.raw_payload_json.find(':', pos);
    if (colon == std::string::npos) return 0;
    std::string tail = env.raw_payload_json.substr(colon + 1);
    try {
      return static_cast<uint8_t>(std::stoul(tail));
    } catch (...) {
      return 0;
    }
  };

  uint8_t decision = extract_num("decision");
  std::string conf = extract_str("confirmation");
  bool disarmed = (context_ ? !context_->is_armed() : true);

  std::string err;
  if (!persistence_->resolve_recovery(decision, env.expected_revision, conf, disarmed, &err)) {
    resp.accepted = false;
    resp.error_code = "ERROR_RESOLVE_RECOVERY_REJECTED";
    resp.error_message = err;
    return resp;
  }

  resp.accepted = true;
  resp.resulting_revision = persistence_->get_recovery_revision();
  resp.response_payload_json = "{\"resolved\":true,\"recovery_revision\":" + std::to_string(persistence_->get_recovery_revision()) + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_list_plan_artifacts(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = true;

  if (!plan_manager_) {
    resp.response_payload_json = "{\"artifacts\":[]}";
    return resp;
  }

  auto list = plan_manager_->list_artifacts();
  std::ostringstream ss;
  ss << "{\"artifacts\":[";
  for (size_t i = 0; i < list.size(); ++i) {
    if (i > 0) ss << ",";
    ss << "{\"id\":\"" << list[i].artifact_id << "\",\"name\":\"" << list[i].safe_name << "\",\"sha256\":\"" << list[i].sha256 << "\"}";
  }
  ss << "]}";
  resp.response_payload_json = ss.str();
  return resp;
}

GatewayResponse FsdGateway::handle_inspect_pad_registry(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = true;

  if (!registry_) {
    resp.response_payload_json = "{\"size\":0}";
    return resp;
  }

  resp.response_payload_json = "{\"size\":" + std::to_string(registry_->size()) + ",\"revision\":" + std::to_string(registry_->get_revision()) + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_inspect_recovery(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = true;

  if (!persistence_) {
    resp.response_payload_json = "{\"recovery_state\":0}";
    return resp;
  }

  const auto & rec = persistence_->get_recovery_status();
  resp.response_payload_json = "{\"recovery_state\":" + std::to_string(rec.state) + ",\"safe_decision_required\":" + (rec.safe_decision_required ? "true" : "false") + "}";
  return resp;
}

GatewayResponse FsdGateway::handle_get_status(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = true;

  std::ostringstream ss;
  ss << "{\"status\":\"OK\"";
  if (context_) {
    ss << ",\"config_state\":" << static_cast<int>(context_->get_state());
    ss << ",\"selection_revision\":" << context_->get_selection_revision();
    ss << ",\"committed_revision\":" << context_->get_committed_revision();
    ss << ",\"armed\":" << (context_->is_armed() ? "true" : "false");
    ss << ",\"locked\":" << (context_->is_locked() ? "true" : "false");
  }
  ss << "}";
  resp.response_payload_json = ss.str();
  return resp;
}

GatewayResponse FsdGateway::handle_get_evidence_manifest(const CommandEnvelope & env)
{
  GatewayResponse resp;
  resp.request_id = env.request_id;
  resp.command = env.command;
  resp.accepted = true;
  resp.response_payload_json = "{\"manifest_id\":\"manifest_default_0\",\"events_count\":0}";
  return resp;
}

}  // namespace full_self_driving::gateway
