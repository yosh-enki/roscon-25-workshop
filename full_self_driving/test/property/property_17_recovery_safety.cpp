#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "full_self_driving/msg/recovery_status.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class RecoverySafetyPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(30303);
    test_dir_ = "/tmp/fsd_test_recovery_" + std::to_string(random_uint(1000, 999999));
    std::filesystem::create_directories(test_dir_);

    paths_.state_directory = test_dir_ + "/state";
    paths_.plan_directory = test_dir_ + "/plans";
    paths_.evidence_directory = test_dir_ + "/evidence";
    paths_.backup_directory = test_dir_ + "/backup";

    std::filesystem::create_directories(paths_.state_directory);
    std::filesystem::create_directories(paths_.plan_directory);
    std::filesystem::create_directories(paths_.evidence_directory);
    std::filesystem::create_directories(paths_.backup_directory);

    persistence_ = std::make_unique<persistence::PersistenceManager>(paths_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::string test_dir_;
  persistence::StoragePaths paths_;
  std::mt19937 rng_;
  std::unique_ptr<persistence::PersistenceManager> persistence_;

  uint32_t random_uint(uint32_t min_val, uint32_t max_val)
  {
    std::uniform_int_distribution<uint32_t> dist(min_val, max_val);
    return dist(rng_);
  }

  persistence::MissionSnapshotRecord create_clean_snapshot()
  {
    persistence::MissionSnapshotRecord snap;
    snap.schema_version = "1.0.0";
    snap.mission_id = "msn_clean_1";
    snap.sortie_id = "srt_clean_1";
    snap.snapshot_revision = 1;
    snap.durable_sequence = 1;
    snap.resolved_config_hash = "hash_cfg_valid_1234567890abcdef1234567890abcdef1234567890abcdef";
    snap.map_id = "kmitl_airfield";
    snap.scenario_id = "default_scenario";
    snap.plan_artifact_id = "art_plan_1";
    snap.working_plan_id = "wp_plan_1";
    snap.working_plan_generation = 1;

    snap.target.marker_id = 0;
    snap.target.dictionary = "DICT_4X4_50";
    snap.target.target_namespace = "aavc2026";

    snap.payload.commanded_state = 1;
    snap.payload.cargo_loaded = true;
    snap.payload.secured = true;
    snap.payload.successful_operations = 0;
    snap.payload.last_operation_id = "";
    snap.payload.last_result = 0;
    snap.payload.unknown_result = false;

    snap.executor.phase = "STANDBY";
    snap.executor.active_action = "NONE";
    snap.executor.durable_sequence = 1;

    snap.checksum = snap.compute_checksum();
    return snap;
  }
};

// Property 17.1: Clean restart with matching config hash and valid records is CLEAR
TEST_F(RecoverySafetyPropertyTest, CleanRestartIsClear)
{
  auto snap = create_clean_snapshot();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, true);
  EXPECT_TRUE(rec_res.is_clear);
  EXPECT_EQ(rec_res.status.state, full_self_driving::msg::RecoveryStatus::STATE_CLEAR);
  EXPECT_FALSE(rec_res.status.safe_decision_required);
  EXPECT_TRUE(rec_res.ambiguity_codes.empty());
}

// Property 17.2: Corrupted or tampered snapshot file detects AMBIGUOUS_SNAPSHOT
TEST_F(RecoverySafetyPropertyTest, CorruptedSnapshotEntersAmbiguousSnapshot)
{
  auto snap = create_clean_snapshot();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  // Corrupt the snapshot on disk
  std::filesystem::path target = std::filesystem::path(paths_.state_directory) / "active_snapshot.json";
  std::ofstream out(target.string(), std::ios::trunc);
  out << "{\"invalid_json_content\": corrupted...";
  out.close();

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, true);
  EXPECT_FALSE(rec_res.is_clear);
  EXPECT_EQ(rec_res.status.state, full_self_driving::msg::RecoveryStatus::STATE_REQUIRED);
  EXPECT_TRUE(rec_res.status.safe_decision_required);

  auto it = std::find(rec_res.ambiguity_codes.begin(), rec_res.ambiguity_codes.end(),
    full_self_driving::msg::RecoveryStatus::AMBIGUOUS_SNAPSHOT);
  EXPECT_NE(it, rec_res.ambiguity_codes.end());
}

// Property 17.3: Config hash mismatch on restart detects AMBIGUOUS_CONFIG_HASH
TEST_F(RecoverySafetyPropertyTest, ConfigHashMismatchEntersAmbiguousConfigHash)
{
  auto snap = create_clean_snapshot();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  std::string new_config_hash = "different_config_hash_value_9999999999999999999999999999999999";
  auto rec_res = persistence_->recover(new_config_hash, true, true);

  EXPECT_FALSE(rec_res.is_clear);
  EXPECT_EQ(rec_res.status.state, full_self_driving::msg::RecoveryStatus::STATE_REQUIRED);

  auto it = std::find(rec_res.ambiguity_codes.begin(), rec_res.ambiguity_codes.end(),
    full_self_driving::msg::RecoveryStatus::AMBIGUOUS_CONFIG_HASH);
  EXPECT_NE(it, rec_res.ambiguity_codes.end());
}

// Property 17.4: Corrupted or missing working plan detects AMBIGUOUS_WORKING_PLAN
TEST_F(RecoverySafetyPropertyTest, MissingWorkingPlanEntersAmbiguousWorkingPlan)
{
  auto snap = create_clean_snapshot();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, false); // is_working_plan_valid = false
  EXPECT_FALSE(rec_res.is_clear);

  auto it = std::find(rec_res.ambiguity_codes.begin(), rec_res.ambiguity_codes.end(),
    full_self_driving::msg::RecoveryStatus::AMBIGUOUS_WORKING_PLAN);
  EXPECT_NE(it, rec_res.ambiguity_codes.end());
}

// Property 17.5: Payload state with unknown_result detects AMBIGUOUS_PAYLOAD
TEST_F(RecoverySafetyPropertyTest, UnknownPayloadResultEntersAmbiguousPayload)
{
  auto snap = create_clean_snapshot();
  snap.payload.unknown_result = true;
  snap.checksum = snap.compute_checksum();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, true);
  EXPECT_FALSE(rec_res.is_clear);

  auto it = std::find(rec_res.ambiguity_codes.begin(), rec_res.ambiguity_codes.end(),
    full_self_driving::msg::RecoveryStatus::AMBIGUOUS_PAYLOAD);
  EXPECT_NE(it, rec_res.ambiguity_codes.end());
}

// Property 17.6: In-flight executor checkpoint on restart detects AMBIGUOUS_EXECUTOR
TEST_F(RecoverySafetyPropertyTest, InFlightExecutorPhaseEntersAmbiguousExecutor)
{
  auto snap = create_clean_snapshot();
  snap.executor.phase = "PRECISION_DESCEND";
  snap.checksum = snap.compute_checksum();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, true);
  EXPECT_FALSE(rec_res.is_clear);

  auto it = std::find(rec_res.ambiguity_codes.begin(), rec_res.ambiguity_codes.end(),
    full_self_driving::msg::RecoveryStatus::AMBIGUOUS_EXECUTOR);
  EXPECT_NE(it, rec_res.ambiguity_codes.end());
}

// Property 17.7: ResolveRecovery resolves state with explicit decision and disarmed confirmation
TEST_F(RecoverySafetyPropertyTest, ResolveRecoveryTransitionsToResolved)
{
  auto snap = create_clean_snapshot();
  snap.payload.unknown_result = true;
  snap.checksum = snap.compute_checksum();
  ASSERT_TRUE(persistence_->commit_snapshot(snap));

  auto rec_res = persistence_->recover(snap.resolved_config_hash, true, true);
  EXPECT_FALSE(rec_res.is_clear);
  EXPECT_EQ(rec_res.status.state, full_self_driving::msg::RecoveryStatus::STATE_REQUIRED);

  // 1. Attempt while armed -> REJECTED
  {
    std::string err;
    bool ok = persistence_->resolve_recovery(
      full_self_driving::msg::RecoveryStatus::DECISION_INSPECT_PAYLOAD_BEFORE_NEXT_SORTIE,
      1, "CONFIRM_RECOVERY", false, &err);
    EXPECT_FALSE(ok);
  }

  // 2. Attempt with revision mismatch -> REJECTED
  {
    std::string err;
    bool ok = persistence_->resolve_recovery(
      full_self_driving::msg::RecoveryStatus::DECISION_INSPECT_PAYLOAD_BEFORE_NEXT_SORTIE,
      99, "CONFIRM_RECOVERY", true, &err);
    EXPECT_FALSE(ok);
  }

  // 3. Attempt with valid disarmed confirmation -> ACCEPTED
  {
    std::string err;
    bool ok = persistence_->resolve_recovery(
      full_self_driving::msg::RecoveryStatus::DECISION_INSPECT_PAYLOAD_BEFORE_NEXT_SORTIE,
      1, "CONFIRM_RECOVERY", true, &err);
    EXPECT_TRUE(ok) << "Error: " << err;

    const auto & status = persistence_->get_recovery_status();
    EXPECT_EQ(status.state, full_self_driving::msg::RecoveryStatus::STATE_RESOLVED);
    EXPECT_FALSE(status.safe_decision_required);
    EXPECT_TRUE(status.has_decision);
    EXPECT_EQ(status.decision, full_self_driving::msg::RecoveryStatus::DECISION_INSPECT_PAYLOAD_BEFORE_NEXT_SORTIE);
    EXPECT_EQ(persistence_->get_recovery_revision(), 2U);
  }
}
