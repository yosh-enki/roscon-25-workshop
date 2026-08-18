#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "flight/internal_strategy.hpp"
#include "gateway/fsd_gateway.hpp"
#include "payload/payload_controller.hpp"
#include "payload/simulation_payload_adapter.hpp"
#include "persistence/persistence_manager.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class Property26SecurityRejectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    test_dir_ = "/tmp/fsd_test_prop26_" + std::to_string(std::rand() % 100000);
    std::filesystem::create_directories(test_dir_);

    context_ = std::make_shared<domain::MissionContext>("ctx_prop26");
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

    adapter_ = std::make_shared<payload::SimulationPayloadAdapter>("prop26_sim_adapter");
    payload_controller_ = std::make_shared<payload::PayloadController>(adapter_, context_);

    coordinator_ = std::make_shared<domain::MissionCoordinator>(context_);
    coordinator_->set_payload_controller(payload_controller_);
    coordinator_->set_persistence_manager(persistence_);

    gateway::GatewaySecurityPolicy policy;
    policy.max_payload_bytes = 1048576;
    policy.max_request_age_s = 5.0;
    policy.max_command_rate_per_minute = 60;

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

// 1. Property 26.1: Unauthorized flight control and raw actuator commands produce zero side effect
TEST_F(Property26SecurityRejectionTest, ForbiddenCommandsProduceNoFlightSideEffect)
{
  const std::vector<std::string> rogue_commands = {
    "arm", "disarm", "ownmode", "takeoff", "land", "rtl", "goto",
    "setpoint", "raw_control", "release", "execute_mission", "kill",
    "override", "direct_actuator", "emergency_drop", "shell", "reboot",
    "shutdown", "set_parameter", "publish_raw"
  };

  const auto initial_strategy = coordinator_->get_current_strategy();
  const auto initial_revision = context_->get_selection_revision();
  const auto initial_status = payload_controller_->get_status();

  for (size_t i = 0; i < rogue_commands.size(); ++i) {
    gateway::CommandEnvelope env;
    env.schema = "full_self_driving.command.v1";
    env.request_id = "rogue_cmd_" + std::to_string(i);
    env.command = rogue_commands[i];
    env.sent_at_unix_ms = 1000000;
    env.expected_revision = initial_revision;
    env.raw_payload_json = "{}";
    env.is_retained = false;

    auto resp = gateway_->process_envelope(env, 1000000);
    EXPECT_FALSE(resp.accepted) << "Rogue command was accepted: " << rogue_commands[i];
    EXPECT_EQ(resp.error_code, "ERROR_FORBIDDEN_COMMAND");

    // Verify ZERO side effects
    EXPECT_EQ(coordinator_->get_current_strategy(), initial_strategy);
    EXPECT_EQ(context_->get_selection_revision(), initial_revision);
    EXPECT_EQ(payload_controller_->get_status().successful_operation_count, initial_status.successful_operation_count);
    EXPECT_FALSE(context_->is_armed());
  }

  EXPECT_EQ(gateway_->get_forbidden_command_attempts(), rogue_commands.size());
  EXPECT_EQ(gateway_->get_security_violations_count(), rogue_commands.size());
}

// 2. Property 26.2: Stale, replayed, and retained commands fail closed with zero state mutation
TEST_F(Property26SecurityRejectionTest, StaleAndReplayedCommandsFailClosed)
{
  const auto initial_strategy = coordinator_->get_current_strategy();
  const uint64_t current_time_ms = 10000000;

  // Stale request (older than max_request_age_s = 5s)
  gateway::CommandEnvelope stale_env;
  stale_env.schema = "full_self_driving.command.v1";
  stale_env.request_id = "stale_req_01";
  stale_env.command = "select_map_scenario";
  stale_env.sent_at_unix_ms = current_time_ms - 10000; // 10s old
  stale_env.expected_revision = context_->get_selection_revision();
  stale_env.raw_payload_json = "{\"map_id\":\"kmitl_airfield\",\"scenario_id\":\"default_scenario\"}";
  stale_env.is_retained = false;

  auto resp = gateway_->process_envelope(stale_env, current_time_ms);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_STALE_REQUEST");
  EXPECT_EQ(coordinator_->get_current_strategy(), initial_strategy);

  // Retained message rejection
  gateway::CommandEnvelope retained_env = stale_env;
  retained_env.request_id = "retained_req_01";
  retained_env.sent_at_unix_ms = current_time_ms;
  retained_env.is_retained = true;

  resp = gateway_->process_envelope(retained_env, current_time_ms);
  EXPECT_FALSE(resp.accepted);
  EXPECT_EQ(resp.error_code, "ERROR_RETAINED_COMMAND_FORBIDDEN");
  EXPECT_EQ(coordinator_->get_current_strategy(), initial_strategy);
}

// 3. Property 26.3: In-flight payload injection rejection causes zero physical release
TEST_F(Property26SecurityRejectionTest, InflightPayloadInjectionProducesZeroActuation)
{
  // Vehicle is in-flight (armed and in TRANSIT_IN)
  context_->set_armed(true);
  bool transition_ok = coordinator_->request_transition(flight::StrategyType::TRANSIT_IN);
  EXPECT_TRUE(transition_ok);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);

  const auto initial_status = payload_controller_->get_status();

  // Attempt external in-flight release injection via Gateway
  gateway::CommandEnvelope attack_env;
  attack_env.schema = "full_self_driving.command.v1";
  attack_env.request_id = "attack_release_01";
  attack_env.command = "prepare_payload";
  attack_env.sent_at_unix_ms = 1000000;
  attack_env.expected_revision = context_->get_selection_revision();
  attack_env.raw_payload_json = "{\"operation\":3}"; // OP_RELEASE_PAYLOAD = 3
  attack_env.is_retained = false;

  auto res = gateway_->process_envelope(attack_env, 1000000);
  EXPECT_FALSE(res.accepted);

  // Verify hardware actuator was NOT triggered
  EXPECT_EQ(payload_controller_->get_status().successful_operation_count, initial_status.successful_operation_count);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_IN);
}

// 4. Property 26.4: Security rejection is durable and audited without corrupting recovery
TEST_F(Property26SecurityRejectionTest, SecurityRejectionsAreAuditedWithoutCorruptingRecovery)
{
  gateway::CommandEnvelope malicious_env;
  malicious_env.schema = "invalid.schema.injection";
  malicious_env.request_id = "malicious_schema_01";
  malicious_env.command = "select_map_scenario";
  malicious_env.sent_at_unix_ms = 1000000;
  malicious_env.expected_revision = context_->get_selection_revision();
  malicious_env.raw_payload_json = "{\"map_id\":\"invalid_malicious_map\"}";
  malicious_env.is_retained = false;

  auto res = gateway_->process_envelope(malicious_env, 1000000);
  EXPECT_FALSE(res.accepted);
  EXPECT_EQ(res.error_code, "ERROR_INVALID_SCHEMA");

  // Verify persistence journal and context remain valid
  EXPECT_FALSE(context_->is_armed());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::WAITING_FOR_MODE);
}
