#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"

using namespace full_self_driving;

class ScopeIsolationPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(98765);
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
  }

  std::mt19937 rng_;
  std::shared_ptr<domain::EngineeringConfig> config_;

  std::string random_choice(const std::vector<std::string> & options)
  {
    std::uniform_int_distribution<size_t> dist(0, options.size() - 1);
    return options[dist(rng_)];
  }
};

// Property 3.1: Monotonic selection revisions on disarmed mutations
TEST_F(ScopeIsolationPropertyTest, Property3_MonotonicSelectionRevision)
{
  domain::MissionContext ctx("ctx_monotonic");
  ctx.set_engineering_config(config_);

  uint64_t expected_rev = ctx.get_selection_revision();
  EXPECT_EQ(expected_rev, 1u);

  // 1. Select map scenario
  EXPECT_TRUE(ctx.select_map_scenario("desert_base", "scenario_alpha", expected_rev));
  expected_rev = ctx.get_selection_revision();
  EXPECT_EQ(expected_rev, 2u);

  // 2. Select target
  domain::TargetIdentity target(42, "DICT_4X4_50", "aavc2026");
  EXPECT_TRUE(ctx.select_target(target, expected_rev));
  expected_rev = ctx.get_selection_revision();
  EXPECT_EQ(expected_rev, 3u);

  // 3. Edit selection
  domain::OperatorSelection sel = ctx.get_selection();
  sel.plan_artifact_id = "plan_artifact_001";
  EXPECT_TRUE(ctx.edit_selection(sel, expected_rev));
  expected_rev = ctx.get_selection_revision();
  EXPECT_EQ(expected_rev, 4u);
}

// Property 3.2: Stale revisions are strictly rejected
TEST_F(ScopeIsolationPropertyTest, Property3_StaleRevisionRejection)
{
  domain::MissionContext ctx("ctx_stale_rev");
  ctx.set_engineering_config(config_);

  uint64_t current_rev = ctx.get_selection_revision();

  // Attempt with stale revision (current_rev - 1)
  std::string err;
  EXPECT_FALSE(ctx.select_map_scenario("new_map", "new_scen", current_rev - 1, &err));
  EXPECT_EQ(ctx.get_selection_revision(), current_rev);

  // Attempt with future/wildcard revision (current_rev + 10)
  EXPECT_FALSE(ctx.select_map_scenario("new_map", "new_scen", current_rev + 10, &err));
  EXPECT_EQ(ctx.get_selection_revision(), current_rev);

  // Attempt with 0 revision
  EXPECT_FALSE(ctx.select_target(domain::TargetIdentity(1, "DICT_4X4_50", "aavc2026"), 0, &err));
  EXPECT_EQ(ctx.get_selection_revision(), current_rev);
}

// Property 3.3: Target constraints enforced against authoritative engineering config
TEST_F(ScopeIsolationPropertyTest, Property3_TargetConstraintsEnforcement)
{
  domain::MissionContext ctx("ctx_target_constraints");
  ctx.set_engineering_config(config_);

  uint64_t rev = ctx.get_selection_revision();
  std::string err;

  // 1. Disallowed dictionary
  domain::TargetIdentity bad_dict(7, "DISALLOWED_DICT_6X6", "aavc2026");
  EXPECT_FALSE(ctx.select_target(bad_dict, rev, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(ctx.get_selection_revision(), rev);

  // 2. Disallowed namespace
  domain::TargetIdentity bad_ns(7, "DICT_4X4_50", "forbidden_namespace");
  EXPECT_FALSE(ctx.select_target(bad_ns, rev, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(ctx.get_selection_revision(), rev);

  // 3. Out of range marker ID
  domain::TargetIdentity bad_id(999999, "DICT_4X4_50", "aavc2026");
  EXPECT_FALSE(ctx.select_target(bad_id, rev, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_EQ(ctx.get_selection_revision(), rev);

  // 4. Valid target succeeds
  domain::TargetIdentity valid_target(7, "DICT_4X4_50", "aavc2026");
  EXPECT_TRUE(ctx.select_target(valid_target, rev, &err));
  EXPECT_EQ(ctx.get_selection_revision(), rev + 1);
}

// Property 3.4: Commit gate requires valid short-lived token and matching revision
TEST_F(ScopeIsolationPropertyTest, Property3_CommitGateRequiresValidToken)
{
  domain::MissionContext ctx("ctx_commit_gate");
  ctx.set_engineering_config(config_);

  domain::TargetIdentity target(7, "DICT_4X4_50", "aavc2026");
  ctx.select_target(target, ctx.get_selection_revision());

  uint64_t current_rev = ctx.get_selection_revision();
  std::string err;

  // 1. Commit without prior validation -> MUST FAIL
  EXPECT_FALSE(ctx.commit("fake_token", current_rev, &err));
  EXPECT_NE(ctx.get_state(), domain::ConfigState::COMMITTED);

  // 2. Validate selection
  auto val_report = ctx.validate_selection(current_rev);
  EXPECT_TRUE(val_report.is_valid);
  EXPECT_FALSE(val_report.token.empty());

  // 3. Commit with wrong token -> MUST FAIL
  EXPECT_FALSE(ctx.commit("wrong_token_1234", current_rev, &err));
  EXPECT_NE(ctx.get_state(), domain::ConfigState::COMMITTED);

  // 4. Commit with valid token and expected revision -> SUCCEEDS
  EXPECT_TRUE(ctx.commit(val_report.token, current_rev, &err));
  EXPECT_EQ(ctx.get_state(), domain::ConfigState::COMMITTED);
  EXPECT_EQ(ctx.get_committed_revision(), current_rev);

  // 5. Replay of already consumed token -> MUST FAIL
  EXPECT_FALSE(ctx.commit(val_report.token, current_rev, &err));
}

// Property 3.5: Multi-scope selection fuzzing isolation
TEST_F(ScopeIsolationPropertyTest, Property3_MultiScopeSelectionFuzzing)
{
  const std::vector<std::string> maps = {"kmitl_airfield", "desert_base", "city_quad", "forest_sector"};
  const std::vector<std::string> scenarios = {"scenario_alpha", "scenario_beta", "scenario_gamma"};
  const std::vector<std::string> dicts = {"DICT_4X4_50", "DICT_4X4_250", "DICT_5X5_50"};
  const std::vector<std::string> namespaces = {"aavc2026", "sar_search", "cargo_delivery"};

  domain::MissionContext ctx("ctx_fuzz");
  ctx.set_engineering_config(config_);

  for (int i = 0; i < 50; ++i) {
    uint64_t rev_before = ctx.get_selection_revision();

    std::string chosen_map = random_choice(maps);
    std::string chosen_scenario = random_choice(scenarios);
    std::string chosen_dict = random_choice(dicts);
    std::string chosen_ns = random_choice(namespaces);
    uint32_t chosen_id = static_cast<uint32_t>(i % 500);

    // Update map/scenario
    ASSERT_TRUE(ctx.select_map_scenario(chosen_map, chosen_scenario, rev_before));
    EXPECT_EQ(ctx.get_selection().map_id, chosen_map);
    EXPECT_EQ(ctx.get_selection().scenario_id, chosen_scenario);
    EXPECT_EQ(ctx.get_selection_revision(), rev_before + 1);

    // Update target
    domain::TargetIdentity tid(chosen_id, chosen_dict, chosen_ns);
    ASSERT_TRUE(ctx.select_target(tid, ctx.get_selection_revision()));
    EXPECT_EQ(ctx.get_selection().target->marker_id, chosen_id);
    EXPECT_EQ(ctx.get_selection().target->dictionary, chosen_dict);
    EXPECT_EQ(ctx.get_selection().target->target_namespace, chosen_ns);
    EXPECT_EQ(ctx.get_selection_revision(), rev_before + 2);

    // Validate and commit
    auto val = ctx.validate_selection(ctx.get_selection_revision());
    ASSERT_TRUE(val.is_valid);
    ASSERT_TRUE(ctx.commit(val.token, ctx.get_selection_revision()));
    EXPECT_EQ(ctx.get_state(), domain::ConfigState::COMMITTED);
    EXPECT_EQ(ctx.get_committed_revision(), rev_before + 2);
  }
}
