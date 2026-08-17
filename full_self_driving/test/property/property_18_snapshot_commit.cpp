#include <gtest/gtest.h>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"
#include "domain/mission_context.hpp"
#include "domain/target_identity.hpp"
#include "persistence/persistence_manager.hpp"
#include "runtime/lifecycle_supervisor.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class SnapshotCommitPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(40404);
    test_dir_ = "/tmp/fsd_test_snap_commit_" + std::to_string(random_uint(1000, 999999));
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
    supervisor_ = std::make_unique<runtime::LifecycleSupervisor>();

    context_ = std::make_unique<domain::MissionContext>("ctx_snap_commit");
    auto cfg = std::make_shared<domain::EngineeringConfig>(domain::EngineeringConfig::create_default_simulation_config());
    context_->set_engineering_config(cfg);
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
  std::unique_ptr<runtime::LifecycleSupervisor> supervisor_;
  std::unique_ptr<domain::MissionContext> context_;

  uint32_t random_uint(uint32_t min_val, uint32_t max_val)
  {
    std::uniform_int_distribution<uint32_t> dist(min_val, max_val);
    return dist(rng_);
  }
};

// Property 18.1: Complete lifecycle activation order (pad_registry -> perception -> evidence -> gateway)
TEST_F(SnapshotCommitPropertyTest, LifecycleActivationFollowsStrictOrder)
{
  auto order = supervisor_->get_activation_order();
  ASSERT_EQ(order.size(), 4U);
  EXPECT_EQ(order[0], "fsd_pad_registry");
  EXPECT_EQ(order[1], "fsd_perception");
  EXPECT_EQ(order[2], "fsd_evidence");
  EXPECT_EQ(order[3], "fsd_gateway");

  std::string failed_node, err;
  ASSERT_TRUE(supervisor_->configure_all(&failed_node, &err)) << "Configure failed: " << err;
  ASSERT_TRUE(supervisor_->activate_all(&failed_node, &err)) << "Activate failed: " << err;

  EXPECT_TRUE(supervisor_->is_all_active());

  // Check transition trace
  const auto & trace = supervisor_->get_transition_trace();
  EXPECT_FALSE(trace.empty());
}

// Property 18.2: Reverse shutdown order on deactivation (gateway -> evidence -> perception -> pad_registry)
TEST_F(SnapshotCommitPropertyTest, ReverseShutdownOrder)
{
  supervisor_->configure_all();
  supervisor_->activate_all();
  supervisor_->clear_transition_trace();

  auto shutdown_order = supervisor_->get_shutdown_order();
  ASSERT_EQ(shutdown_order.size(), 4U);
  EXPECT_EQ(shutdown_order[0], "fsd_gateway");
  EXPECT_EQ(shutdown_order[1], "fsd_evidence");
  EXPECT_EQ(shutdown_order[2], "fsd_perception");
  EXPECT_EQ(shutdown_order[3], "fsd_pad_registry");

  std::string failed_node, err;
  ASSERT_TRUE(supervisor_->deactivate_all(&failed_node, &err));
  EXPECT_FALSE(supervisor_->is_all_active());
}

// Property 18.3: Lifecycle transition failure stops progression and deactivates already-active nodes in reverse order
TEST_F(SnapshotCommitPropertyTest, TransitionFailureReversesActivatedNodes)
{
  supervisor_->configure_all();
  // Inject fault during activation of fsd_evidence (3rd node in order)
  supervisor_->set_fault_injection("fsd_evidence", "activate");

  std::string failed_node, err;
  bool ok = supervisor_->activate_all(&failed_node, &err);
  EXPECT_FALSE(ok);
  EXPECT_EQ(failed_node, "fsd_evidence");

  // Previously activated nodes (pad_registry, perception) should have been deactivated back to INACTIVE
  EXPECT_EQ(supervisor_->get_node_state("fsd_pad_registry"), runtime::LifecycleState::INACTIVE);
  EXPECT_EQ(supervisor_->get_node_state("fsd_perception"), runtime::LifecycleState::INACTIVE);
  EXPECT_EQ(supervisor_->get_node_state("fsd_gateway"), runtime::LifecycleState::INACTIVE);
  EXPECT_FALSE(supervisor_->is_all_active());
}

// Property 18.4: Snapshot commit occurs strictly after validation and durable write
TEST_F(SnapshotCommitPropertyTest, SnapshotCommitOrderingAndReconciliation)
{
  // 1. Select map, scenario, target
  ASSERT_TRUE(context_->select_map_scenario("kmitl_airfield", "default_scenario", 1));
  domain::TargetIdentity target{0, "DICT_4X4_50", "aavc2026"};
  ASSERT_TRUE(context_->select_target(target, 2));

  // 2. Validate selection
  auto val_report = context_->validate_selection(3);
  ASSERT_TRUE(val_report.is_valid);
  EXPECT_FALSE(val_report.token.empty());

  // 3. Commit mission context
  std::string err;
  ASSERT_TRUE(context_->commit(val_report.token, 3, &err)) << "Commit failed: " << err;
  EXPECT_EQ(context_->get_state(), domain::ConfigState::COMMITTED);
  EXPECT_EQ(context_->get_committed_revision(), 3U);

  // 4. Durably commit snapshot via persistence
  persistence::MissionSnapshotRecord snap;
  snap.schema_version = "1.0.0";
  snap.snapshot_revision = context_->get_committed_revision();
  snap.durable_sequence = 1;
  snap.resolved_config_hash = context_->get_resolved_config_hash();
  snap.map_id = "kmitl_airfield";
  snap.scenario_id = "default_scenario";
  snap.target = target;
  snap.checksum = snap.compute_checksum();

  ASSERT_TRUE(persistence_->commit_snapshot(snap, &err));
  EXPECT_EQ(persistence_->get_durable_sequence(), 1U);

  // 5. Restart reconciliation
  auto rec_res = persistence_->recover(context_->get_resolved_config_hash(), true, true);
  EXPECT_TRUE(rec_res.is_clear);
  EXPECT_EQ(rec_res.status.state, full_self_driving::msg::RecoveryStatus::STATE_CLEAR);
}
