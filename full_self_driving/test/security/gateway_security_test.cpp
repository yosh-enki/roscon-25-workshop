#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "domain/target_identity.hpp"
#include "gateway/fsd_gateway.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class GatewaySecurityIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = "/tmp/fsd_test_gw_sec_" + std::to_string(std::rand() % 100000);
    std::filesystem::create_directories(test_dir_);

    context_ = std::make_shared<domain::MissionContext>("ctx_gw_sec");
    auto cfg = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    context_->set_engineering_config(cfg);

    plan_manager_ = std::make_shared<runtime::PlanManager>(test_dir_ + "/plans");
    registry_ = std::make_shared<registry::PadRegistry>();

    persistence::StoragePaths paths;
    paths.state_directory = test_dir_ + "/state";
    paths.plan_directory = test_dir_ + "/plans";
    paths.evidence_directory = test_dir_ + "/evidence";
    paths.backup_directory = test_dir_ + "/backup";
    persistence_ = std::make_shared<persistence::PersistenceManager>(paths);

    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("gw_sec_sim_adapter");
    payload_controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);

    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);
    coordinator_->set_payload_controller(payload_controller_);
    coordinator_->set_persistence_manager(persistence_);

    gateway::GatewaySecurityPolicy policy;
    policy.max_payload_bytes = 1048576; // 1 MiB
    policy.max_request_age_s = 5.0;     // 5 seconds
    policy.max_command_rate_per_minute = 100;

    gateway_ = std::make_shared<gateway::FsdGateway>(
      policy, context_, plan_manager_, registry_, persistence_);
    gateway_->set_payload_controller(payload_controller_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::string test_dir_;
  std::shared_ptr<domain::MissionContext> context_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> registry_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<payload::SimulationPayloadAdapter> adapter_;
  std::shared_ptr<payload::PayloadController> payload_controller_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<gateway::FsdGateway> gateway_;
};

// 1. Clock Skew and Drift Rejection
TEST_F(GatewaySecurityIntegrationTest, ClockSkewAndDriftRejection)
{
  const uint64_t current_time_ms = 5000000;
  const auto initial_revision = context_->get_selection_revision();

  // Timestamp in far future (+1000s)
  gateway::CommandEnvelope future_env;
  future_env.schema = "full_self_driving.command.v1";
  future_env.request_id = "skew_req_future_01";
  future_env.command = "select_map_scenario";
  future_env.sent_at_unix_ms = current_time_ms + 1000000;
  future_env.expected_revision = initial_revision;
  future_env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"default_scenario\"}";
  future_env.is_retained = false;

  auto resp_future = gateway_->process_envelope(future_env, current_time_ms);
  EXPECT_FALSE(resp_future.accepted);
  EXPECT_EQ(resp_future.error_code, "ERROR_CLOCK_SKEW");
  EXPECT_EQ(context_->get_selection_revision(), initial_revision);

  // Timestamp in far past (-1000s)
  gateway::CommandEnvelope past_env = future_env;
  past_env.request_id = "stale_req_past_01";
  past_env.sent_at_unix_ms = current_time_ms - 1000000;

  auto resp_past = gateway_->process_envelope(past_env, current_time_ms);
  EXPECT_FALSE(resp_past.accepted);
  EXPECT_EQ(resp_past.error_code, "ERROR_STALE_REQUEST");
  EXPECT_EQ(context_->get_selection_revision(), initial_revision);
}

// 2. Retained MQTT Command Rejection
TEST_F(GatewaySecurityIntegrationTest, RetainedMessageRejection)
{
  const uint64_t current_time_ms = 1000000;
  gateway::CommandEnvelope env;
  env.schema = "full_self_driving.command.v1";
  env.request_id = "retained_attack_01";
  env.command = "clear_pad_registry";
  env.sent_at_unix_ms = current_time_ms;
  env.expected_revision = context_->get_selection_revision();
  env.raw_payload_json = "{}";
  env.is_retained = true;

  auto resp = gateway_->process_envelope(env, current_time_ms);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_RETAINED_COMMAND_FORBIDDEN");
}

// 3. Fuzzing: Malformed JSON, Syntax Errors, and Oversized Payloads
TEST_F(GatewaySecurityIntegrationTest, FuzzingMalformedAndOversizedPayloads)
{
  const uint64_t current_time_ms = 1000000;

  // Oversized JSON (> 1 MiB limit)
  std::string huge_payload(2 * 1024 * 1024, 'A');
  auto resp_huge = gateway_->process_command_json(huge_payload, false, current_time_ms);
  EXPECT_FALSE(resp_huge.accepted);
  EXPECT_EQ(resp_huge.error_code, "ERROR_PAYLOAD_TOO_LARGE");

  // Malformed garbage JSON
  std::vector<std::string> malformed_samples = {
    "",
    "not a json",
    "{\"schema\": }",
    "{\"schema\": \"full_self_driving.command.v1\", \"command\": ",
    "{\"request_id\": 12345}",
    "{\"schema\": \"full_self_driving.command.v1\", \"request_id\": \"\", \"command\": \"get_status\"}"
  };

  for (const auto & sample : malformed_samples) {
    auto resp = gateway_->process_command_json(sample, false, current_time_ms);
    EXPECT_FALSE(resp.accepted);
  }
}

// 4. Command Injection & Path Traversal Fuzzing
TEST_F(GatewaySecurityIntegrationTest, CommandAndPathInjectionFuzzing)
{
  const uint64_t current_time_ms = 1000000;
  const auto initial_revision = context_->get_selection_revision();

  std::vector<std::string> injection_vectors = {
    "$(rm -rf /)",
    "; reboot; ",
    "| cat /etc/shadow",
    "../../../../etc/passwd",
    "' OR '1'='1",
    "<script>alert(1)</script>",
    "../../../dev/urandom"
  };

  for (size_t i = 0; i < injection_vectors.size(); ++i) {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "inject_req_" + std::to_string(i);
    env.command = "upload_plan_artifact";
    env.sent_at_unix_ms = current_time_ms;
    env.expected_revision = initial_revision;
    env.raw_payload_json = "{\"artifact_name\":\"" + injection_vectors[i] + "\",\"plan_json_content\":\"{}\"}";
    env.is_retained = false;

    auto resp = gateway_->process_envelope(env, current_time_ms);
    // Even if parsed, upload fails validation safely without system execution
    EXPECT_FALSE(context_->is_armed());
  }

  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);
  EXPECT_FALSE(context_->is_armed());
}

// 5. Broker ACL & Rogue Flight Command Rejection
TEST_F(GatewaySecurityIntegrationTest, BrokerAclForbiddenFlightCommands)
{
  const uint64_t current_time_ms = 1000000;
  const std::vector<std::string> rogue_commands = {
    "arm", "disarm", "takeoff", "land", "rtl", "setpoint", "raw_control",
    "kill", "override", "direct_actuator", "shell", "reboot", "shutdown"
  };

  for (size_t i = 0; i < rogue_commands.size(); ++i) {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "acl_violation_" + std::to_string(i);
    env.command = rogue_commands[i];
    env.sent_at_unix_ms = current_time_ms;
    env.expected_revision = 0;
    env.raw_payload_json = "{}";
    env.is_retained = false;

    auto resp = gateway_->process_envelope(env, current_time_ms);
    EXPECT_FALSE(resp.accepted);
    EXPECT_EQ(resp.error_code, "ERROR_FORBIDDEN_COMMAND");
  }

  // Verify zero side effects
  EXPECT_FALSE(context_->is_armed());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);
}
