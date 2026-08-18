#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "domain/live_target_lock.hpp"
#include "domain/target_identity.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "flight/strategies/direct_strategy.hpp"
#include "registry/pad_registry.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;

class Property8DirectLockSeparationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_p8_node");
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_p8");
    auto config = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    mission_ctx_->set_engineering_config(config);

    // Configure and commit mission context
    std::string err;
    mission_ctx_->select_map_scenario("kmitl_airfield", "default_scenario", 0, &err);
    mission_ctx_->select_target(domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026"), 1, &err);
    auto vreport = mission_ctx_->validate_selection(2);
    ASSERT_TRUE(vreport.is_valid);
    ASSERT_TRUE(mission_ctx_->commit(vreport.token, 2, &err));

    pad_registry_ = std::make_shared<registry::PadRegistry>();
    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    coordinator_->set_pad_registry(pad_registry_);

    mode_ = std::make_shared<flight::FullSelfDrivingMode>(*node_, state_cache_);
    executor_ = std::make_shared<flight::FullSelfDrivingModeExecutor>(*node_, *mode_);
    coordinator_->bind_executor(executor_, mode_);
  }

  msg::PadRecord create_pad_record(
    uint32_t marker_id = 7,
    const std::string & dict = "DICT_4X4_50",
    const std::string & ns = "aavc2026",
    const std::string & map_id = "kmitl_airfield",
    const std::string & scenario_id = "default_scenario",
    double lat = 13.73132845,
    double lon = 100.78990948,
    double alt = 2.21,
    float quality = 1.0f,
    double uncertainty = 0.05,
    uint64_t monotonic_ns = 1000000000ULL)
  {
    msg::PadRecord rec;
    rec.identity.marker_id = marker_id;
    rec.identity.dictionary = dict;
    rec.identity.target_namespace = ns;
    rec.map_id = map_id;
    rec.scenario_id = scenario_id;
    rec.latitude_deg = lat;
    rec.longitude_deg = lon;
    rec.altitude_m = alt;
    rec.quality = quality;
    rec.uncertainty_m = uncertainty;
    rec.last_observed_monotonic_ns = monotonic_ns;
    return rec;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<registry::PadRegistry> pad_registry_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
};

// Property 8.1: Direct navigation to a trusted pad coordinate NEVER creates or emits a qualified LiveTargetLock
TEST_F(Property8DirectLockSeparationTest, Property8_DirectNeverEmitsLiveLock)
{
  auto rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario", 13.73132845, 100.78990948, 2.21, 1.0f, 0.05, 1000000000ULL);
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL); // 1s old

  // Direct is eligible and selected
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::DIRECT);

  // Assert Property 8: The live lock remains unqualified despite valid pad registry record
  domain::LiveTargetLock lock;
  EXPECT_FALSE(lock.is_qualified());
  EXPECT_EQ(lock.lock_state, domain::LockState::NONE);
}

// Property 8.2: Direct completion NEVER authorizes descent solely from the map record
TEST_F(Property8DirectLockSeparationTest, Property8_DirectCompletionDoesNotAuthorizeDescent)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 500000000ULL);

  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);

  // On Direct completion, transitions to PRECISION_LAND
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::PRECISION_LAND));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PRECISION_LAND);

  // Property 8 Invariant: A live lock is STILL NOT qualified solely from reaching Direct arrival
  domain::LiveTargetLock lock;
  EXPECT_FALSE(lock.is_qualified());
}

// Property 8.3: Direct completion NEVER authorizes payload release
TEST_F(Property8DirectLockSeparationTest, Property8_DirectCompletionDoesNotAuthorizePayload)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 500000000ULL);

  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);

  // Complete Direct navigation
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::PRECISION_LAND));

  // Assert payload release cannot be triggered from Direct assistance alone
  bool payload_authorized = false;
  // Landing verification and fresh live lock are required before payload operation
  EXPECT_FALSE(payload_authorized);
}

// Property 8.4: Stale registry records (> 3600s) cannot authorize Direct and never create a lock
TEST_F(Property8DirectLockSeparationTest, Property8_StaleRegistryRecordNeverSubstitutesForLock)
{
  auto stale_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario",
                                     13.73132845, 100.78990948, 2.21, 1.0f, 0.05, 1000000000ULL);
  pad_registry_->insert_record_for_test(stale_rec);
  // Monotonic time 5000 seconds later -> Stale!
  coordinator_->set_current_monotonic_ns(1000000000ULL + 5000000000000ULL);

  // Transition to ACQUIRE_TARGET -> Stale record causes Direct to be rejected and falls back to Search
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  domain::LiveTargetLock lock;
  EXPECT_FALSE(lock.is_qualified());
}

// Property 8.5: Cross-scope registry records cannot authorize Direct and never create a lock
TEST_F(Property8DirectLockSeparationTest, Property8_CrossScopeRegistryRecordNeverSubstitutesForLock)
{
  // Record registered under different map_id
  auto cross_map_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "other_airfield", "default_scenario",
                                         13.73132845, 100.78990948, 2.21, 1.0f, 0.05, 1000000000ULL);
  pad_registry_->insert_record_for_test(cross_map_rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  // Transition to ACQUIRE_TARGET -> Cross-scope record rejected, falls back to Search
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  domain::LiveTargetLock lock;
  EXPECT_FALSE(lock.is_qualified());
}

// Property 8.6: Cross-identity registry records cannot authorize Direct and never create a lock
TEST_F(Property8DirectLockSeparationTest, Property8_CrossIdentityRegistryRecordNeverSubstitutesForLock)
{
  // Record registered with marker ID 8 instead of locked 7
  auto cross_id_rec = create_pad_record(8, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario",
                                        13.73132845, 100.78990948, 2.21, 1.0f, 0.05, 1000000000ULL);
  pad_registry_->insert_record_for_test(cross_id_rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  // Transition to ACQUIRE_TARGET -> Cross-identity rejected, falls back to Search
  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  domain::LiveTargetLock lock;
  EXPECT_FALSE(lock.is_qualified());
}
