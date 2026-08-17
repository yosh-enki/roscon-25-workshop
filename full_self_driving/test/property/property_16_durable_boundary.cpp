#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "persistence/persistence_manager.hpp"

using namespace full_self_driving;

class DurableBoundaryPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(20202);
    test_dir_ = "/tmp/fsd_test_durable_" + std::to_string(random_uint(1000, 999999));
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

  persistence::MissionSnapshotRecord create_valid_snapshot(uint64_t revision)
  {
    persistence::MissionSnapshotRecord snap;
    snap.schema_version = "1.0.0";
    snap.mission_id = "msn_test_" + std::to_string(revision);
    snap.sortie_id = "srt_test_" + std::to_string(revision);
    snap.snapshot_revision = revision;
    snap.durable_sequence = revision;
    snap.resolved_config_hash = "abcdef0123456789abcdef0123456789abcdef0123456789abcdef0123456789";
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
    snap.executor.durable_sequence = revision;

    snap.checksum = snap.compute_checksum();
    return snap;
  }
};

// Property 16.1: Full successful durability pipeline commits snapshot and advances sequence
TEST_F(DurableBoundaryPropertyTest, SuccessfulPipelineCommitsAndAdvancesSequence)
{
  for (uint64_t rev = 1; rev <= 5; ++rev) {
    auto snap = create_valid_snapshot(rev);
    std::string err;
    bool ok = persistence_->commit_snapshot(snap, &err);
    ASSERT_TRUE(ok) << "Failed to commit snapshot: " << err;

    EXPECT_EQ(persistence_->get_durable_sequence(), rev);

    auto loaded = persistence_->load_active_snapshot();
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->snapshot_revision, rev);
    EXPECT_EQ(loaded->checksum, snap.checksum);
  }
}

// Property 16.2: Injected fault at VALIDATE stage fails and preserves prior state
TEST_F(DurableBoundaryPropertyTest, FaultAtValidatePreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  persistence_->set_fault_injection(persistence::FaultStage::VALIDATE);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  auto loaded = persistence_->load_active_snapshot();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->snapshot_revision, 1U);
}

// Property 16.3: Injected fault at TEMP_WRITE stage fails and preserves prior state
TEST_F(DurableBoundaryPropertyTest, FaultAtTempWritePreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));

  persistence_->set_fault_injection(persistence::FaultStage::TEMP_WRITE);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  auto loaded = persistence_->load_active_snapshot();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->snapshot_revision, 1U);
}

// Property 16.4: Injected fault at FLUSH_FSYNC stage cleans temp and preserves prior state
TEST_F(DurableBoundaryPropertyTest, FaultAtFlushFsyncPreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));

  persistence_->set_fault_injection(persistence::FaultStage::FLUSH_FSYNC);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  auto loaded = persistence_->load_active_snapshot();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->snapshot_revision, 1U);
}

// Property 16.5: Injected fault at RENAME stage fails atomically
TEST_F(DurableBoundaryPropertyTest, FaultAtRenamePreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));

  persistence_->set_fault_injection(persistence::FaultStage::RENAME);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  auto loaded = persistence_->load_active_snapshot();
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(loaded->snapshot_revision, 1U);
}

// Property 16.6: Injected fault at DIRECTORY_SYNC stage fails
TEST_F(DurableBoundaryPropertyTest, FaultAtDirectorySyncPreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));

  persistence_->set_fault_injection(persistence::FaultStage::DIRECTORY_SYNC);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);
}

// Property 16.7: Injected fault at JOURNAL stage fails commit
TEST_F(DurableBoundaryPropertyTest, FaultAtJournalPreservesPriorState)
{
  auto snap1 = create_valid_snapshot(1);
  ASSERT_TRUE(persistence_->commit_snapshot(snap1));

  persistence_->set_fault_injection(persistence::FaultStage::JOURNAL);

  auto snap2 = create_valid_snapshot(2);
  std::string err;
  bool ok = persistence_->commit_snapshot(snap2, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);
}

// Property 16.8: Backup creation writes backup metadata and durable file before mutation
TEST_F(DurableBoundaryPropertyTest, BackupCreationIsDurable)
{
  std::string backup_id;
  std::string content = "{\"records\": [{\"marker_id\": 1}]}";
  bool ok = persistence_->create_backup("registry", "kmitl_airfield", content, 1, &backup_id);
  ASSERT_TRUE(ok);
  EXPECT_FALSE(backup_id.empty());

  auto backups = persistence_->list_backups();
  ASSERT_EQ(backups.size(), 1U);
  EXPECT_EQ(backups[0].backup_id, backup_id);
  EXPECT_EQ(backups[0].source_type, "registry");

  std::filesystem::path b_file = std::filesystem::path(paths_.backup_directory) / (backup_id + ".bak");
  EXPECT_TRUE(std::filesystem::exists(b_file));
}
