#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/plan_manager.hpp"
#include "registry/pad_registry.hpp"

namespace full_self_driving::gateway
{

struct GatewaySecurityPolicy
{
  uint64_t max_payload_bytes{8388608}; // 8 MiB
  double max_request_age_s{30.0};
  uint32_t max_command_rate_per_minute{120};
};

struct CommandEnvelope
{
  std::string schema;
  std::string request_id;
  std::string command;
  uint64_t sent_at_unix_ms{0};
  uint64_t expected_revision{0};
  std::string raw_payload_json;
  bool is_retained{false};
};

struct GatewayResponse
{
  bool accepted{false};
  std::string request_id;
  std::string command;
  uint64_t resulting_revision{0};
  std::string response_payload_json;
  std::string error_code;
  std::string error_message;
  uint8_t severity{0};
};

class FsdGateway
{
public:
  explicit FsdGateway(
    const GatewaySecurityPolicy & policy = GatewaySecurityPolicy(),
    std::shared_ptr<domain::MissionContext> context = nullptr,
    std::shared_ptr<runtime::PlanManager> plan_manager = nullptr,
    std::shared_ptr<registry::PadRegistry> registry = nullptr,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr);
  ~FsdGateway() = default;

  void set_policy(const GatewaySecurityPolicy & policy);
  const GatewaySecurityPolicy & get_policy() const { return policy_; }

  void set_mission_context(std::shared_ptr<domain::MissionContext> ctx);
  void set_plan_manager(std::shared_ptr<runtime::PlanManager> pm);
  void set_pad_registry(std::shared_ptr<registry::PadRegistry> reg);
  void set_persistence_manager(std::shared_ptr<persistence::PersistenceManager> pm);

  GatewayResponse process_command_json(
    const std::string & json_envelope,
    bool is_retained = false,
    uint64_t current_unix_ms = 0);

  GatewayResponse process_envelope(
    const CommandEnvelope & env,
    uint64_t current_unix_ms = 0);

  bool is_command_allowed(const std::string & command) const;
  bool is_command_forbidden(const std::string & command) const;

  uint64_t get_forbidden_command_attempts() const { return forbidden_command_attempts_; }
  uint64_t get_security_violations_count() const { return security_violations_count_; }

  void reset_idempotency_cache();

private:
  GatewayResponse handle_select_map_scenario(const CommandEnvelope & env);
  GatewayResponse handle_select_target_identity(const CommandEnvelope & env);
  GatewayResponse handle_select_plan_artifact(const CommandEnvelope & env);
  GatewayResponse handle_create_or_select_working_plan(const CommandEnvelope & env);
  GatewayResponse handle_reset_working_plan(const CommandEnvelope & env);
  GatewayResponse handle_upload_plan_artifact(const CommandEnvelope & env);
  GatewayResponse handle_prepare_payload(const CommandEnvelope & env);
  GatewayResponse handle_clear_pad_registry(const CommandEnvelope & env);
  GatewayResponse handle_validate_mission_context(const CommandEnvelope & env);
  GatewayResponse handle_commit_mission_context(const CommandEnvelope & env);
  GatewayResponse handle_resolve_recovery(const CommandEnvelope & env);

  // Inspection
  GatewayResponse handle_list_plan_artifacts(const CommandEnvelope & env);
  GatewayResponse handle_inspect_pad_registry(const CommandEnvelope & env);
  GatewayResponse handle_inspect_recovery(const CommandEnvelope & env);
  GatewayResponse handle_get_status(const CommandEnvelope & env);
  GatewayResponse handle_get_evidence_manifest(const CommandEnvelope & env);

  mutable std::mutex mutex_;
  GatewaySecurityPolicy policy_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> registry_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;

  uint64_t forbidden_command_attempts_{0};
  uint64_t security_violations_count_{0};
  std::map<std::string, GatewayResponse> idempotency_cache_;

  static const std::set<std::string> ALLOWED_COMMANDS;
  static const std::set<std::string> FORBIDDEN_COMMANDS;
};

}  // namespace full_self_driving::gateway
