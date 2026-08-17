#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"

using namespace full_self_driving;

class ArmedImmutabilityPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(54321);
    config_ = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
  }

  std::mt19937 rng_;
  std::shared_ptr<domain::EngineeringConfig> config_;

  std::unique_ptr<domain::MissionContext> create_committed_and_locked_context()
  {
    auto ctx = std::make_unique<domain::MissionContext>("ctx_test");
    std::string err;
    bool cfg_ok = ctx->set_engineering_config(config_, &err);
    EXPECT_TRUE(cfg_ok) << "Config error: " << err;

    domain::TargetIdentity target(7, "DICT_4X4_50", "aavc2026");
    bool target_ok = ctx->select_target(target, ctx->get_selection_revision(), &err);
    EXPECT_TRUE(target_ok) << "Target error: " << err;

    auto val = ctx->validate_selection(ctx->get_selection_revision());
    EXPECT_TRUE(val.is_valid);
    EXPECT_FALSE(val.token.empty());

    bool commit_ok = ctx->commit(val.token, ctx->get_selection_revision(), &err);
    EXPECT_TRUE(commit_ok) << "Commit error: " << err;

    bool ready = ctx->check_readiness(true, true, true);
    EXPECT_TRUE(ready);

    bool lock_ok = ctx->lock("mission_alpha", "sortie_001", &err);
    EXPECT_TRUE(lock_ok) << "Lock error: " << err;

    return ctx;
  }
};

// Property 2.1: Committed context captures matching canonical hash
TEST_F(ArmedImmutabilityPropertyTest, Property2_CommittedContextHashConsistency)
{
  auto ctx = std::make_unique<domain::MissionContext>("ctx_hash_test");
  ctx->set_engineering_config(config_);

  domain::TargetIdentity target(15, "DICT_4X4_50", "aavc2026");
  ctx->select_target(target, ctx->get_selection_revision());

  auto val = ctx->validate_selection(ctx->get_selection_revision());
  ASSERT_TRUE(val.is_valid);

  ctx->commit(val.token, ctx->get_selection_revision());
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::COMMITTED);
  EXPECT_EQ(ctx->get_resolved_config_hash(), config_->compute_canonical_hash());
  EXPECT_EQ(ctx->get_selection().resolved_config_hash, config_->compute_canonical_hash());
}

// Property 2.2: Locked context strictly rejects all mutations
TEST_F(ArmedImmutabilityPropertyTest, Property2_LockedContextRejectsMutations)
{
  auto ctx = create_committed_and_locked_context();
  ASSERT_TRUE(ctx->is_locked());
  ASSERT_EQ(ctx->get_state(), domain::ConfigState::LOCKED);

  std::string original_hash = ctx->get_resolved_config_hash();
  uint64_t original_rev = ctx->get_committed_revision();
  std::string original_mission = ctx->get_mission_id();
  std::string original_sortie = ctx->get_sortie_id();

  std::string err;

  // 1. Attempt edit_selection
  domain::OperatorSelection new_sel;
  new_sel.map_id = "desert_base";
  EXPECT_FALSE(ctx->edit_selection(new_sel, ctx->get_selection_revision(), &err));
  EXPECT_FALSE(err.empty());

  // 2. Attempt select_map_scenario
  EXPECT_FALSE(ctx->select_map_scenario("new_map", "new_scenario", ctx->get_selection_revision(), &err));

  // 3. Attempt select_target
  domain::TargetIdentity new_target(99, "DICT_4X4_50", "aavc2026");
  EXPECT_FALSE(ctx->select_target(new_target, ctx->get_selection_revision(), &err));

  // 4. Attempt validate_selection
  auto val = ctx->validate_selection(ctx->get_selection_revision());
  EXPECT_FALSE(val.is_valid);

  // 5. Attempt commit
  EXPECT_FALSE(ctx->commit("some_token", ctx->get_selection_revision(), &err));

  // 6. Attempt set_engineering_config
  auto new_cfg = std::make_shared<domain::EngineeringConfig>(
    domain::EngineeringConfig::create_default_simulation_config());
  new_cfg->deployment_id = "hacked_deployment";
  EXPECT_FALSE(ctx->set_engineering_config(new_cfg, &err));

  // 7. Attempt second lock
  EXPECT_FALSE(ctx->lock("other_mission", "other_sortie", &err));

  // Verify all state remained 100% immutable
  EXPECT_TRUE(ctx->is_locked());
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::LOCKED);
  EXPECT_EQ(ctx->get_resolved_config_hash(), original_hash);
  EXPECT_EQ(ctx->get_committed_revision(), original_rev);
  EXPECT_EQ(ctx->get_mission_id(), original_mission);
  EXPECT_EQ(ctx->get_sortie_id(), original_sortie);
}

// Property 2.3: Armed state prevents unlocking
TEST_F(ArmedImmutabilityPropertyTest, Property2_ArmedVehicleCannotUnlock)
{
  auto ctx = create_committed_and_locked_context();
  ctx->set_armed(true);
  EXPECT_TRUE(ctx->is_armed());

  std::string err;

  // Unlocking while vehicle is armed MUST FAIL
  EXPECT_FALSE(ctx->unlock(false, &err));
  EXPECT_FALSE(ctx->unlock(true, &err));  // Even if caller claims disarmed, internal armed flag rejects
  EXPECT_TRUE(ctx->is_locked());
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::LOCKED);

  // Once confirmed disarmed, unlocking succeeds
  ctx->set_armed(false);
  EXPECT_TRUE(ctx->unlock(true, &err));
  EXPECT_FALSE(ctx->is_locked());
  EXPECT_EQ(ctx->get_state(), domain::ConfigState::STANDBY);
}

// Property 2.4: Fuzzed mutation sequence invariant under lock
TEST_F(ArmedImmutabilityPropertyTest, Property2_FuzzedMutationSequenceInvariant)
{
  auto ctx = create_committed_and_locked_context();
  std::string locked_hash = ctx->get_resolved_config_hash();
  uint64_t committed_rev = ctx->get_committed_revision();

  std::uniform_int_distribution<int> action_dist(0, 5);

  for (int i = 0; i < 100; ++i) {
    int action = action_dist(rng_);
    std::string err;

    switch (action) {
      case 0: {
        domain::OperatorSelection sel;
        sel.map_id = "random_map_" + std::to_string(i);
        EXPECT_FALSE(ctx->edit_selection(sel, committed_rev, &err));
        break;
      }
      case 1: {
        EXPECT_FALSE(ctx->select_map_scenario("rand_map", "rand_scen", committed_rev, &err));
        break;
      }
      case 2: {
        domain::TargetIdentity tid(static_cast<uint32_t>(i), "DICT_4X4_50", "aavc2026");
        EXPECT_FALSE(ctx->select_target(tid, committed_rev, &err));
        break;
      }
      case 3: {
        EXPECT_FALSE(ctx->commit("tok_" + std::to_string(i), committed_rev, &err));
        break;
      }
      case 4: {
        auto cfg = std::make_shared<domain::EngineeringConfig>(
          domain::EngineeringConfig::create_default_simulation_config());
        cfg->safety.max_altitude_m = 999.0;
        EXPECT_FALSE(ctx->set_engineering_config(cfg, &err));
        break;
      }
      case 5: {
        auto report = ctx->validate_selection(committed_rev);
        EXPECT_FALSE(report.is_valid);
        break;
      }
    }

    // Invariant holds after every single attempt
    ASSERT_TRUE(ctx->is_locked());
    ASSERT_EQ(ctx->get_state(), domain::ConfigState::LOCKED);
    ASSERT_EQ(ctx->get_resolved_config_hash(), locked_hash);
    ASSERT_EQ(ctx->get_committed_revision(), committed_rev);
  }
}
