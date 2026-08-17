#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"

using namespace full_self_driving;

class ReadinessPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
  }

  std::shared_ptr<domain::EngineeringConfig> config_;

  std::unique_ptr<domain::MissionContext> create_committed_context()
  {
    auto ctx = std::make_unique<domain::MissionContext>("ctx_readiness_test");
    ctx->set_engineering_config(config_);
    domain::TargetIdentity target(7, "DICT_4X4_50", "aavc2026");
    ctx->select_target(target, ctx->get_selection_revision());
    auto report = ctx->validate_selection(ctx->get_selection_revision());
    EXPECT_TRUE(report.is_valid);
    EXPECT_TRUE(ctx->commit(report.token, ctx->get_selection_revision()));
    EXPECT_EQ(ctx->get_state(), domain::ConfigState::COMMITTED);
    return ctx;
  }
};

// Property 9.1: Uncommitted context strictly blocks readiness
TEST_F(ReadinessPropertyTest, Property9_UncommittedContextBlocksReadiness)
{
  domain::MissionContext ctx("ctx_uncommitted");
  std::vector<std::string> missing;

  // Initial state is STANDBY, not COMMITTED
  EXPECT_FALSE(ctx.check_readiness(true, true, true, &missing));
  EXPECT_NE(ctx.get_state(), domain::ConfigState::READY_FOR_OWNMODE);
  EXPECT_FALSE(missing.empty());

  bool found_commit_gate = false;
  for (const auto & gate : missing) {
    if (gate.find("committed") != std::string::npos) {
      found_commit_gate = true;
    }
  }
  EXPECT_TRUE(found_commit_gate);
}

// Property 9.2: Missing target identity blocks readiness
TEST_F(ReadinessPropertyTest, Property9_MissingTargetBlocksReadiness)
{
  domain::MissionContext ctx("ctx_no_target");
  ctx.set_engineering_config(config_);

  // Validate without target -> should fail
  auto val = ctx.validate_selection(ctx.get_selection_revision());
  EXPECT_FALSE(val.is_valid);

  std::vector<std::string> missing;
  EXPECT_FALSE(ctx.check_readiness(true, true, true, &missing));
  EXPECT_NE(ctx.get_state(), domain::ConfigState::READY_FOR_OWNMODE);
}

// Property 9.3: Individual dependency failures withdraw readiness
TEST_F(ReadinessPropertyTest, Property9_IndividualDependencyFailuresBlockReadiness)
{
  // 1. PX4 transport failure
  {
    auto ctx = create_committed_context();
    std::vector<std::string> missing;
    EXPECT_FALSE(ctx->check_readiness(false, true, true, &missing));
    EXPECT_NE(ctx->get_state(), domain::ConfigState::READY_FOR_OWNMODE);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "PX4 transport is not ready");
  }

  // 2. Storage failure
  {
    auto ctx = create_committed_context();
    std::vector<std::string> missing;
    EXPECT_FALSE(ctx->check_readiness(true, false, true, &missing));
    EXPECT_NE(ctx->get_state(), domain::ConfigState::READY_FOR_OWNMODE);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "Durable storage is not healthy");
  }

  // 3. Component health failure
  {
    auto ctx = create_committed_context();
    std::vector<std::string> missing;
    EXPECT_FALSE(ctx->check_readiness(true, true, false, &missing));
    EXPECT_NE(ctx->get_state(), domain::ConfigState::READY_FOR_OWNMODE);
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], "Component health check failed");
  }
}

// Property 9.4: Multi-fault combinations are completely enumerated
TEST_F(ReadinessPropertyTest, Property9_MultiFaultCombinationsEnumerated)
{
  auto ctx = create_committed_context();
  std::vector<std::string> missing;

  // All dependencies fail
  EXPECT_FALSE(ctx->check_readiness(false, false, false, &missing));
  EXPECT_EQ(missing.size(), 3u);

  bool px4_found = false, storage_found = false, health_found = false;
  for (const auto & gate : missing) {
    if (gate.find("PX4") != std::string::npos) px4_found = true;
    if (gate.find("storage") != std::string::npos || gate.find("Storage") != std::string::npos) storage_found = true;
    if (gate.find("health") != std::string::npos || gate.find("Health") != std::string::npos) health_found = true;
  }
  EXPECT_TRUE(px4_found);
  EXPECT_TRUE(storage_found);
  EXPECT_TRUE(health_found);
}

// Property 9.5: Complete and healthy context achieves READY_FOR_OWNMODE
TEST_F(ReadinessPropertyTest, Property9_AllGatesPassAchievesReadyForOwnmode)
{
  auto ctx = create_committed_context();
  std::vector<std::string> missing;

  EXPECT_TRUE(ctx->check_readiness(true, true, true, &missing));
  EXPECT_TRUE(missing.empty());
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::READY_FOR_OWNMODE);

  // Subsequent failure withdraws readiness back to COMMITTED
  EXPECT_FALSE(ctx->check_readiness(false, true, true, &missing));
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::COMMITTED);
  EXPECT_EQ(missing.size(), 1u);
}
