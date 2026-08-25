#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "domain/mission_context.hpp"
#include "domain/mission_coordinator.hpp"
#include "domain/live_target_lock.hpp"
#include "domain/target_identity.hpp"
#include "domain/working_plan.hpp"
#include "flight/full_self_driving_mode.hpp"
#include "flight/full_self_driving_mode_executor.hpp"
#include "flight/strategies/direct_strategy.hpp"
#include "flight/strategies/search_strategy.hpp"
#include "registry/pad_registry.hpp"
#include "runtime/plan_manager.hpp"
#include "adapters/px4_state_cache.hpp"

using namespace full_self_driving;

class AcquisitionBranchTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("test_acq_branch_node");
    context_ = std::make_unique<px4_ros2::Context>(*node_);
    state_cache_ = std::make_shared<adapters::Px4StateCache>(*context_);
    mission_ctx_ = std::make_shared<domain::MissionContext>("ctx_acq_test");
    auto config = std::make_shared<domain::EngineeringConfig>(
      domain::EngineeringConfig::create_default_simulation_config());
    mission_ctx_->set_engineering_config(config);

    // Commit valid mission selection
    std::string err;
    mission_ctx_->select_map_scenario("kmitl_airfield", "default_scenario", 0, &err);
    mission_ctx_->select_target(domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026"), 1, &err);
    auto vreport = mission_ctx_->validate_selection(2);
    ASSERT_TRUE(vreport.is_valid);
    ASSERT_TRUE(mission_ctx_->commit(vreport.token, 2, &err));

    plan_manager_ = std::make_shared<runtime::PlanManager>("/tmp/fsd_acq_plans");
    pad_registry_ = std::make_shared<registry::PadRegistry>();

    coordinator_ = std::make_shared<domain::MissionCoordinator>(mission_ctx_);
    coordinator_->set_plan_manager(plan_manager_);
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

  domain::WorkingPlan create_valid_working_plan()
  {
    domain::CanonicalSearchRoute route;
    route.default_altitude_m = 15.0f;
    route.cruise_speed_m_s = 5.0f;
    route.waypoints.push_back(domain::SearchWaypoint{13.73132845, 100.78990948, 15.0, 0});
    route.waypoints.push_back(domain::SearchWaypoint{13.73078947, 100.78783793, 15.0, 1});

    domain::WorkingPlan wp("wp_acq_test", "plan_art_1", "kmitl_airfield", "default_scenario", "sha_dummy", route);
    return wp;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<px4_ros2::Context> context_;
  std::shared_ptr<adapters::Px4StateCache> state_cache_;
  std::shared_ptr<domain::MissionContext> mission_ctx_;
  std::shared_ptr<runtime::PlanManager> plan_manager_;
  std::shared_ptr<registry::PadRegistry> pad_registry_;
  std::shared_ptr<domain::MissionCoordinator> coordinator_;
  std::shared_ptr<flight::FullSelfDrivingMode> mode_;
  std::shared_ptr<flight::FullSelfDrivingModeExecutor> executor_;
};

// 1. Trusted Direct navigation is selected when trusted pad record and gates pass
TEST_F(AcquisitionBranchTest, Branch_TrustedDirect_Selected)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL); // 1s old
  coordinator_->set_battery_percentage(95.0);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::DIRECT);

  const auto & trace = coordinator_->get_transition_trace();
  bool found_direct_event = false;
  for (const auto & entry : trace) {
    if (entry.find("FLY-004") != std::string::npos || entry.find("EVT_ACQUISITION_DIRECT_SELECTED") != std::string::npos) {
      found_direct_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_direct_event);
}

// 1b. Target NOT found in registry data falls back to Search (FLY-005)
TEST_F(AcquisitionBranchTest, Branch_TargetNotFoundInRegistry_FallsBackToSearch)
{
  // PadRegistry is completely empty (target has not been found in data)
  // No record inserted
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  coordinator_->set_battery_percentage(95.0);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::SEARCH);

  const auto & trace = coordinator_->get_transition_trace();
  bool found_search_event = false;
  for (const auto & entry : trace) {
    if (entry.find("FLY-005") != std::string::npos || entry.find("EVT_ACQUISITION_SEARCH_SELECTED") != std::string::npos) {
      found_search_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_search_event);
}

// 2. Stale registry record falls back to Search
TEST_F(AcquisitionBranchTest, Branch_StaleRecord_FallsBackToSearch)
{
  auto stale_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario",
                                     13.73132845, 100.78990948, 2.21, 1.0f, 0.05, 1000000000ULL);
  pad_registry_->insert_record_for_test(stale_rec);
  // Stale age: 4000 seconds > 3600s max
  coordinator_->set_current_monotonic_ns(1000000000ULL + 4000000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::SEARCH);

  const auto & trace = coordinator_->get_transition_trace();
  bool found_search_event = false;
  for (const auto & entry : trace) {
    if (entry.find("FLY-005") != std::string::npos || entry.find("EVT_ACQUISITION_SEARCH_SELECTED") != std::string::npos) {
      found_search_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_search_event);
}

// 3. Cross-scope registry record falls back to Search
TEST_F(AcquisitionBranchTest, Branch_CrossScope_FallsBackToSearch)
{
  auto cross_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "different_map", "default_scenario");
  pad_registry_->insert_record_for_test(cross_rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 4. Cross-identity registry record falls back to Search
TEST_F(AcquisitionBranchTest, Branch_CrossIdentity_FallsBackToSearch)
{
  auto cross_id_rec = create_pad_record(99, "DICT_4X4_50", "aavc2026");
  pad_registry_->insert_record_for_test(cross_id_rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 5. Low quality record falls back to Search
TEST_F(AcquisitionBranchTest, Branch_LowQuality_FallsBackToSearch)
{
  auto low_q_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario",
                                     13.73132845, 100.78990948, 2.21, -0.5f, 0.05);
  pad_registry_->insert_record_for_test(low_q_rec);
  coordinator_->set_minimum_record_quality(0.0f);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 6. Excessive position uncertainty falls back to Search
TEST_F(AcquisitionBranchTest, Branch_UncertaintyExceeded_FallsBackToSearch)
{
  auto uncertain_rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario",
                                         13.73132845, 100.78990948, 2.21, 1.0f, 100.0); // 100m > 50m max
  pad_registry_->insert_record_for_test(uncertain_rec);
  coordinator_->set_max_record_uncertainty_m(50.0);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 7. Energy gate failure falls back to Search
TEST_F(AcquisitionBranchTest, Branch_EnergyGateFailure_FallsBackToSearch)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  // Low battery: 10% < 20% min
  coordinator_->set_battery_percentage(10.0);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 8. No trusted record AND no valid working plan -> Fails closed to HOLD
TEST_F(AcquisitionBranchTest, Branch_NoRecordAndNoPlan_FailsClosedToHold)
{
  // PadRegistry empty, no custom search plan, no working plan in PlanManager
  coordinator_->reset_custom_direct();
  coordinator_->reset_custom_search();

  std::string err;
  bool res = coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET, &err);
  EXPECT_FALSE(res);
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::HOLD);
  EXPECT_FALSE(err.empty());

  const auto & trace = coordinator_->get_transition_trace();
  bool found_hold_event = false;
  for (const auto & entry : trace) {
    if (entry.find("ACQUISITION_FAILED_HOLD") != std::string::npos) {
      found_hold_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_hold_event);
}

// 9. Direct navigation completion transitions to PRECISION_LAND (FLY-006 / EVT_DIRECT_COMPLETE)
TEST_F(AcquisitionBranchTest, Branch_DirectCompletion_TransitionsToPrecisionLand)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);

  // Complete Direct navigation
  EXPECT_TRUE(coordinator_->handle_direct_complete());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::PRECISION_LAND);

  const auto & trace = coordinator_->get_transition_trace();
  bool found_complete_event = false;
  for (const auto & entry : trace) {
    if (entry.find("FLY-006") != std::string::npos || entry.find("EVT_DIRECT_COMPLETE") != std::string::npos) {
      found_complete_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_complete_event);
}

// 10. Direct in-flight timeout/fallback transitions to SEARCH (FLY-007 / EVT_DIRECT_FALLBACK)
TEST_F(AcquisitionBranchTest, Branch_DirectFallback_TransitionsToSearch)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  ASSERT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);

  // In-flight fallback
  EXPECT_TRUE(coordinator_->handle_direct_fallback("direct timeout in flight"));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::SEARCH);

  const auto & trace = coordinator_->get_transition_trace();
  bool found_fallback_event = false;
  for (const auto & entry : trace) {
    if (entry.find("FLY-007") != std::string::npos || entry.find("EVT_DIRECT_FALLBACK") != std::string::npos) {
      found_fallback_event = true;
      break;
    }
  }
  EXPECT_TRUE(found_fallback_event);
}

// 11. Direct disabled by policy selects Search immediately
TEST_F(AcquisitionBranchTest, Branch_DirectDisabled_SelectsSearchImmediately)
{
  auto rec = create_pad_record();
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  coordinator_->set_direct_enabled(false);

  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::ACQUIRE_TARGET));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
}

// 12. CompleteGridFirst: Target lock during Search does NOT preempt Search
TEST_F(AcquisitionBranchTest, Branch_CompleteGridFirst_LockDoesNotPreemptSearch)
{
  coordinator_->set_search_policy("complete_grid_first");
  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  // Send qualified live target lock
  domain::LiveTargetLock lock;
  lock.lock_state = domain::LockState::QUALIFIED;
  lock.identity = domain::TargetIdentity(7, "DICT_4X4_50", "aavc2026");
  coordinator_->handle_target_lock_update(lock);

  // In complete_grid_first, search is NOT preempted to PRECISION_LAND
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::SEARCH);
}

// 13. CompleteGridFirst: Search completion transitions to Direct if target was recorded in PadRegistry
TEST_F(AcquisitionBranchTest, Branch_CompleteGridFirst_SearchCompleteTransitionsToDirect)
{
  coordinator_->set_search_policy("complete_grid_first");
  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  // Pad is recorded into PadRegistry during survey
  auto rec = create_pad_record(7, "DICT_4X4_50", "aavc2026", "kmitl_airfield", "default_scenario", 13.73132845, 100.78990948, 15.0);
  pad_registry_->insert_record_for_test(rec);
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  coordinator_->set_battery_percentage(85.0);

  // Handle search completed
  EXPECT_TRUE(coordinator_->handle_search_completed());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::DIRECT);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::DIRECT);
}

// 14. CompleteGridFirst: Search completion transitions to TRANSIT_OUT if target was NOT seen
TEST_F(AcquisitionBranchTest, Branch_CompleteGridFirst_SearchCompleteMissingTargetTransitionsToTransitOut)
{
  coordinator_->set_search_policy("complete_grid_first");
  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  // PadRegistry is empty (target was never seen)
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  coordinator_->set_battery_percentage(85.0);

  // Handle search completed -> Should transition to TRANSIT_OUT (Egress corridor)
  EXPECT_TRUE(coordinator_->handle_search_completed());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::TRANSIT_OUT);
}


