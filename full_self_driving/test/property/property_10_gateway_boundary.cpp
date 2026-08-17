#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"
#include "gateway/fsd_gateway.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/plan_manager.hpp"
#include "registry/pad_registry.hpp"

using namespace full_self_driving;

class GatewayBoundaryPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(10101);
    test_dir_ = "/tmp/fsd_test_gateway_" + std::to_string(random_uint(1000, 999999));
    std::filesystem::create_directories(test_dir_);

    context_ = std::make_shared<domain::MissionContext>("ctx_test_gw");
    auto cfg = std::make_shared<domain::EngineeringConfig>(domain::EngineeringConfig::create_default_simulation_config());
    context_->set_engineering_config(cfg);

    plan_manager_ = std::make_shared<runtime::PlanManager>(test_dir_ + "/plans");
    registry_ = std::make_shared<registry::PadRegistry>();

    persistence::StoragePaths paths;
    paths.state_directory = test_dir_ + "/state";
    paths.plan_directory = test_dir_ + "/plans";
    paths.evidence_directory = test_dir_ + "/evidence";
    paths.backup_directory = test_dir_ + "/backup";
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

    gateway::GatewaySecurityPolicy policy;
    policy.max_payload_bytes = 8388608;
    policy.max_request_age_s = 30.0;
    policy.max_command_rate_per_minute = 120;

    gateway_ = std::make_unique<gateway::FsdGateway>(
      policy, context_, plan_manager_, registry_, persistence_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::string test_dir_;
  std::mt19937 rng_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> registry_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::unique_ptr<gateway::FsdGateway> gateway_;

  uint32_t random_uint(uint32_t min_val, uint32_t max_val)
  {
    std::uniform_int_distribution<uint32_t> dist(min_val, max_val);
    return dist(rng_);
  }
};

// Property 10.1: Reject all forbidden flight, arming, setpoint, and raw-control commands
TEST_F(GatewayBoundaryPropertyTest, RejectsForbiddenFlightAndControlCommands)
{
  const std::vector<std::string> forbidden_cmds = {
    "arm", "disarm", "ownmode", "takeoff", "land", "rtl", "goto",
    "setpoint", "raw_control", "release", "execute_mission", "kill",
    "override", "direct_actuator", "emergency_drop", "shell", "reboot",
    "shutdown", "set_parameter", "publish_raw"
  };

  for (size_t i = 0; i < forbidden_cmds.size(); ++i) {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "req_forbidden_" + std::to_string(i);
    env.command = forbidden_cmds[i];
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 1;
    env.raw_payload_json = "{}";
    env.is_retained = false;

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_FALSE(resp.accepted) << "Forbidden command was accepted: " << forbidden_cmds[i];
    EXPECT_EQ(resp.error_code, "ERROR_FORBIDDEN_COMMAND");
    EXPECT_EQ(resp.severity, 3U);
  }

  EXPECT_EQ(gateway_->get_forbidden_command_attempts(), forbidden_cmds.size());
  EXPECT_EQ(gateway_->get_security_violations_count(), forbidden_cmds.size());
}

// Property 10.2: Reject retained MQTT commands
TEST_F(GatewayBoundaryPropertyTest, RejectsRetainedCommands)
{
  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "req_retained_1";
  env.command = "select_map_scenario";
  env.sent_at_unix_ms = 1000000;
  env.expected_revision = 1;
  env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"default_scenario\"}";
  env.is_retained = true; // Retained!

  auto resp = gateway_->process_envelope(env, 1000000);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_RETAINED_COMMAND_FORBIDDEN");
}

// Property 10.3: Reject invalid or unsupported schemas
TEST_F(GatewayBoundaryPropertyTest, RejectsInvalidSchemas)
{
  const std::vector<std::string> invalid_schemas = {
    "full_self_driving.command.v2",
    "ros2.command.v1",
    "",
    "unknown_schema",
    "qgc.mission.v1"
  };

  for (size_t i = 0; i < invalid_schemas.size(); ++i) {
    gateway::CommandEnvelope env;
    env.schema = invalid_schemas[i];
    env.request_id = "req_schema_" + std::to_string(i);
    env.command = "select_map_scenario";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 1;
    env.raw_payload_json = "{}";
    env.is_retained = false;

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_FALSE(resp.accepted);
    EXPECT_EQ(resp.error_code, "ERROR_INVALID_SCHEMA");
  }
}

// Property 10.4: Reject empty or overlong request IDs
TEST_F(GatewayBoundaryPropertyTest, RejectsInvalidRequestIds)
{
  // 1. Empty request ID
  {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "";
    env.command = "select_map_scenario";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 1;
    env.raw_payload_json = "{}";

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_FALSE(resp.accepted);
    EXPECT_EQ(resp.error_code, "ERROR_INVALID_REQUEST_ID");
  }

  // 2. Overlong request ID (> 64 chars)
  {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = std::string(65, 'x');
    env.command = "select_map_scenario";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 1;
    env.raw_payload_json = "{}";

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_FALSE(resp.accepted);
    EXPECT_EQ(resp.error_code, "ERROR_INVALID_REQUEST_ID");
  }
}

// Property 10.5: Reject stale requests exceeding maximum request age
TEST_F(GatewayBoundaryPropertyTest, RejectsStaleRequests)
{
  uint64_t sent_time = 1000000;
  // 31 seconds later (> 30s limit)
  uint64_t current_time = sent_time + 31000;

  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "req_stale_1";
  env.command = "select_map_scenario";
  env.sent_at_unix_ms = sent_time;
  env.expected_revision = 1;
  env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"default_scenario\"}";

  auto resp = gateway_->process_envelope(env, current_time);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_STALE_REQUEST");
}

// Property 10.6: Idempotent command processing returns identical cached response
TEST_F(GatewayBoundaryPropertyTest, IdempotentCommandProcessingReturnsCachedResponse)
{
  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "req_idempotent_1";
  env.command = "select_map_scenario";
  env.sent_at_unix_ms = 1000000;
  env.expected_revision = 1;
  env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"default_scenario\"}";

  auto resp1 = gateway_->process_envelope(env, 1000000);
  ASSERT_TRUE(resp1.accepted);
  EXPECT_EQ(resp1.resulting_revision, 2U);

  // Send identical request ID again
  auto resp2 = gateway_->process_envelope(env, 1000000);
  EXPECT_TRUE(resp2.accepted);
  EXPECT_EQ(resp1.resulting_revision, resp2.resulting_revision);
  EXPECT_EQ(resp1.response_payload_json, resp2.response_payload_json);

  // Selection revision should NOT have incremented a second time
  EXPECT_EQ(context_->get_selection_revision(), 2U);
}

// Property 10.7: Reject mutations while armed or locked
TEST_F(GatewayBoundaryPropertyTest, RejectsMutationsWhileArmedOrLocked)
{
  context_->set_armed(true);

  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "req_armed_mutation_1";
  env.command = "select_map_scenario";
  env.sent_at_unix_ms = 1000000;
  env.expected_revision = 1;
  env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"scenario_2\"}";

  auto resp = gateway_->process_envelope(env, 1000000);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_SELECTION_REJECTED");
}

// Property 10.8: Allowlisted preparation and inspection commands succeed
TEST_F(GatewayBoundaryPropertyTest, AllowedPreparationAndInspectionCommandsSucceed)
{
  // 1. select_target_identity
  {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "req_target_1";
    env.command = "select_target_identity";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 1;
    env.raw_payload_json = "{\"marker_id\":0,\"dictionary\":\"DICT_4X4_50\",\"target_namespace\":\"aavc2026\"}";

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_TRUE(resp.accepted);
    EXPECT_EQ(resp.resulting_revision, 2U);
  }

  // 2. list_plan_artifacts (inspection)
  {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "req_list_plans";
    env.command = "list_plan_artifacts";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 0;
    env.raw_payload_json = "{}";

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_TRUE(resp.accepted);
    EXPECT_NE(resp.response_payload_json.find("\"artifacts\":"), std::string::npos);
  }

  // 3. get_status (inspection)
  {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "req_get_status";
    env.command = "get_status";
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = 0;
    env.raw_payload_json = "{}";

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_TRUE(resp.accepted);
    EXPECT_NE(resp.response_payload_json.find("\"status\":\"OK\""), std::string::npos);
  }
}
