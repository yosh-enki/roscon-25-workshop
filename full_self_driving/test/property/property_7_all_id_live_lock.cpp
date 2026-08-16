#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "domain/live_target_lock.hpp"
#include "perception/target_coordinator.hpp"
#include "registry/pad_registry.hpp"

using namespace full_self_driving;

class AllIdLiveLockSeparationPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    policy_.minimum_quality = 0.3f;
    policy_.maximum_pose_age_s = 0.5;
    policy_.minimum_consecutive_observations = 2;
    policy_.maximum_position_uncertainty = 5.0;
    policy_.spatial_consistency_radius_m = 2.0;
    policy_.target_loss_timeout_s = 1.5;

    coordinator_ = std::make_unique<perception::TargetCoordinator>(
      policy_, "kmitl_airfield", "default_scenario");
  }

  domain::TargetLockPolicy policy_;
  std::unique_ptr<perception::TargetCoordinator> coordinator_;

  msg::AllIdObservation create_observation(
    uint32_t marker_id,
    const std::string & dict = "DICT_4X4_50",
    const std::string & ns = "aavc2026",
    double x = 0.0, double y = 0.0, double z = 5.0,
    float quality = 0.9f,
    uint8_t obs_state = msg::AllIdObservation::QUALITY_ACCEPTED,
    const std::string & calib_hash = "valid_calib_hash",
    const std::string & map_id = "kmitl_airfield",
    const std::string & scenario_id = "default_scenario")
  {
    msg::AllIdObservation obs;
    obs.identity.marker_id = marker_id;
    obs.identity.dictionary = dict;
    obs.identity.target_namespace = ns;
    obs.map_id = map_id;
    obs.scenario_id = scenario_id;
    obs.pose_frame = "camera_frame";
    obs.pose.position.x = x;
    obs.pose.position.y = y;
    obs.pose.position.z = z;
    obs.pose.orientation.w = 1.0;
    obs.quality = quality;
    obs.observation_state = obs_state;
    obs.calibration_sha256 = calib_hash;
    obs.covariance = {
      0.01, 0, 0, 0, 0, 0,
      0, 0.01, 0, 0, 0, 0,
      0, 0, 0.04, 0, 0, 0,
      0, 0, 0, 0.01, 0, 0,
      0, 0, 0, 0, 0.01, 0,
      0, 0, 0, 0, 0, 0.01
    };
    return obs;
  }
};

// Property 7.1: All-ID observations alone DO NOT create a live lock without a selected target
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_NoLockWithoutSelectedTarget)
{
  msg::AllIdObservationBatch batch;
  batch.map_id = "kmitl_airfield";
  batch.scenario_id = "default_scenario";
  batch.observations.push_back(create_observation(0));
  batch.observations.push_back(create_observation(1));
  batch.observations.push_back(create_observation(7));

  uint64_t t0 = 1000000000;
  auto lock = coordinator_->process_observation_batch(batch, t0);

  EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  EXPECT_EQ(lock.consecutive_observations, 0u);
  EXPECT_FALSE(lock.is_qualified());
}

// Property 7.2: Target mismatch (ID, dictionary, namespace, or scope) never qualifies
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_MismatchNeverQualifies)
{
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");
  coordinator_->set_selected_target(selected);

  uint64_t t0 = 1000000000;

  // 1. Different marker ID (e.g. ID 8)
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(8, "DICT_4X4_50", "aavc2026"));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }

  // 2. Different dictionary
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_5X5_50", "aavc2026"));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }

  // 3. Different namespace
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_4X4_50", "other_ns"));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }

  // 4. Different map scope
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 0, 0, 5, 0.9f,
      msg::AllIdObservation::QUALITY_ACCEPTED, "valid_hash", "desert_base", "default_scenario"));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }
}

// Property 7.3: Exact match with consecutive observations properly transitions CANDIDATE -> QUALIFIED
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_ConsecutiveQualification)
{
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");
  coordinator_->set_selected_target(selected);

  msg::AllIdObservationBatch batch;
  batch.observations.push_back(create_observation(1));  // Other marker
  batch.observations.push_back(create_observation(7));  // Matching marker

  uint64_t t0 = 1000000000;
  auto lock1 = coordinator_->process_observation_batch(batch, t0);

  // 1st observation: reaches CANDIDATE (min_consecutive=2)
  EXPECT_EQ(lock1.lock_state, domain::LockState::CANDIDATE);
  EXPECT_EQ(lock1.consecutive_observations, 1u);
  EXPECT_FALSE(lock1.is_qualified());

  // 2nd observation: reaches QUALIFIED
  uint64_t t1 = t0 + 100000000;  // 100ms later
  auto lock2 = coordinator_->process_observation_batch(batch, t1);

  EXPECT_EQ(lock2.lock_state, domain::LockState::QUALIFIED);
  EXPECT_EQ(lock2.consecutive_observations, 2u);
  EXPECT_TRUE(lock2.is_qualified());
  EXPECT_EQ(lock2.identity.marker_id, 7u);
  EXPECT_EQ(lock2.identity.dictionary, "DICT_4X4_50");
  EXPECT_EQ(lock2.identity.target_namespace, "aavc2026");
}

// Property 7.4: Quality and Calibration Gates
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_QualityAndCalibrationGates)
{
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");
  coordinator_->set_selected_target(selected);

  uint64_t t0 = 1000000000;

  // 1. Rejected state
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 0, 0, 5, 0.9f,
      msg::AllIdObservation::QUALITY_REJECTED));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }

  // 2. Low quality below threshold
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 0, 0, 5, 0.05f));  // < 0.3
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }

  // 3. Missing calibration hash
  {
    msg::AllIdObservationBatch batch;
    batch.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 0, 0, 5, 0.9f,
      msg::AllIdObservation::QUALITY_ACCEPTED, ""));
    auto lock = coordinator_->process_observation_batch(batch, t0);
    EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  }
}

// Property 7.5: Freshness timeout transitions (QUALIFIED -> STALE -> LOST)
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_FreshnessTimeouts)
{
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");
  coordinator_->set_selected_target(selected);

  msg::AllIdObservationBatch batch;
  batch.observations.push_back(create_observation(7));

  uint64_t t0 = 1000000000;
  coordinator_->process_observation_batch(batch, t0);
  uint64_t t1 = t0 + 100000000;
  auto lock = coordinator_->process_observation_batch(batch, t1);
  ASSERT_EQ(lock.lock_state, domain::LockState::QUALIFIED);

  // Advance by 600ms (> max_pose_age_s=0.5s, < loss_timeout=1.5s)
  uint64_t t_stale = t1 + 600000000;
  auto lock_stale = coordinator_->check_freshness(t_stale);
  EXPECT_EQ(lock_stale.lock_state, domain::LockState::STALE);

  // Advance by 2.0s (> loss_timeout=1.5s)
  uint64_t t_lost = t1 + 2000000000;
  auto lock_lost = coordinator_->check_freshness(t_lost);
  EXPECT_EQ(lock_lost.lock_state, domain::LockState::LOST);
  EXPECT_EQ(lock_lost.consecutive_observations, 0u);
}

// Property 7.6: Spatial consistency gate rejects sudden large jumps from qualifying immediately
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_SpatialConsistency)
{
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");
  coordinator_->set_selected_target(selected);

  // Observation 1 at (0, 0, 5)
  msg::AllIdObservationBatch batch1;
  batch1.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 0.0, 0.0, 5.0));
  uint64_t t0 = 1000000000;
  auto lock1 = coordinator_->process_observation_batch(batch1, t0);
  EXPECT_EQ(lock1.consecutive_observations, 1u);
  EXPECT_EQ(lock1.lock_state, domain::LockState::CANDIDATE);

  // Observation 2 jump to (50.0, 50.0, 5.0) -> jump > 2.0m spatial radius
  msg::AllIdObservationBatch batch2;
  batch2.observations.push_back(create_observation(7, "DICT_4X4_50", "aavc2026", 50.0, 50.0, 5.0));
  uint64_t t1 = t0 + 100000000;
  auto lock2 = coordinator_->process_observation_batch(batch2, t1);

  // Resets consecutive to 1, stays CANDIDATE rather than QUALIFIED
  EXPECT_EQ(lock2.consecutive_observations, 1u);
  EXPECT_EQ(lock2.lock_state, domain::LockState::CANDIDATE);
}

// Property 7.7: Registry existence does NOT substitute for or create a live visual lock
TEST_F(AllIdLiveLockSeparationPropertyTest, Property7_RegistryDoesNotImplyLiveLock)
{
  registry::PadRegistry registry;
  domain::TargetIdentity selected(7, "DICT_4X4_50", "aavc2026");

  // Insert pad record into registry
  msg::PadRecord rec;
  rec.map_id = "kmitl_airfield";
  rec.scenario_id = "default_scenario";
  rec.identity = selected.to_msg();
  rec.latitude_deg = 13.727;
  rec.longitude_deg = 100.778;
  rec.altitude_m = 15.0;
  rec.quality = 1.0f;
  rec.uncertainty_m = 0.01;
  rec.calibration_sha256 = "hash";
  registry.insert_record_for_test(rec);

  // Verify registry has it
  auto lookup_res = registry.lookup(selected, "kmitl_airfield", "default_scenario");
  ASSERT_TRUE(lookup_res.has_value());

  // Verify coordinator without visual observations has NO live lock
  coordinator_->set_selected_target(selected);
  auto lock = coordinator_->get_current_lock();
  EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
  EXPECT_EQ(lock.consecutive_observations, 0u);
  EXPECT_FALSE(lock.is_qualified());
}
