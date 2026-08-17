#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"
#include "domain/working_plan.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class WorkingPlanPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(98765);
  }

  std::mt19937 rng_;

  double random_double(double min_val, double max_val)
  {
    std::uniform_real_distribution<double> dist(min_val, max_val);
    return dist(rng_);
  }

  domain::CanonicalSearchRoute generate_random_route(size_t num_waypoints = 10)
  {
    domain::CanonicalSearchRoute route;
    route.default_altitude_m = 15.0f;
    route.cruise_speed_m_s = 5.0f;

    for (size_t i = 0; i < num_waypoints; ++i) {
      domain::SearchWaypoint wp;
      wp.latitude_deg = random_double(13.7, 13.8);
      wp.longitude_deg = random_double(100.7, 100.8);
      wp.altitude_m = 15.0;
      wp.source_index = static_cast<uint32_t>(i);
      route.waypoints.push_back(wp);
    }
    route.canonical_route_sha256 = domain::PlanParser::compute_canonical_route_hash(route);
    return route;
  }
};

// Property 5.1: Working-plan initialization starts with generation 1, 0% progress, and preserved source hash
TEST_F(WorkingPlanPropertyTest, InitializationCorrectness)
{
  runtime::PlanManager manager;
  auto route = generate_random_route(12);
  std::string json = domain::PlanPrinter::print(route);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  auto art = manager.upload_artifact("test.plan", bytes, 0);
  ASSERT_TRUE(art.has_value());

  auto wp = manager.create_or_select_working_plan(
    art->artifact_id, "kmitl_airfield", "default_scenario", 0);
  ASSERT_TRUE(wp.has_value());

  EXPECT_EQ(wp->get_generation(), 1U);
  EXPECT_EQ(wp->get_source_artifact_id(), art->artifact_id);
  EXPECT_EQ(wp->get_source_artifact_sha256(), art->sha256);
  EXPECT_EQ(wp->get_canonical_route_sha256(), art->route.canonical_route_sha256);
  EXPECT_EQ(wp->get_map_id(), "kmitl_airfield");
  EXPECT_EQ(wp->get_scenario_id(), "default_scenario");

  const auto & cp = wp->get_checkpoint();
  EXPECT_EQ(cp.generation, 1U);
  EXPECT_EQ(cp.next_source_index, 0U);
  EXPECT_FALSE(cp.has_checkpoint_position);
  EXPECT_EQ(cp.completed_waypoints, 0U);
  EXPECT_EQ(cp.total_waypoints, 12U);
  EXPECT_FLOAT_EQ(cp.progress_percent, 0.0f);
  EXPECT_EQ(cp.checkpoint_reason, "INITIAL");
  EXPECT_EQ(wp->get_state(), domain::WorkingPlanState::READY);
}

// Property 5.2: Checkpoint progression advances state, sequence, and progress percentage
TEST_F(WorkingPlanPropertyTest, CheckpointProgression)
{
  runtime::PlanManager manager;
  auto route = generate_random_route(10);
  std::string json = domain::PlanPrinter::print(route);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  auto art = manager.upload_artifact("test_progress.plan", bytes, 0);
  ASSERT_TRUE(art.has_value());

  auto wp = manager.create_or_select_working_plan(
    art->artifact_id, "kmitl_airfield", "default_scenario", 0);
  ASSERT_TRUE(wp.has_value());

  for (uint32_t step = 1; step <= 10; ++step) {
    domain::SearchCheckpointData cp;
    cp.next_source_index = step;
    cp.completed_waypoints = step;
    cp.total_waypoints = 10;
    cp.has_checkpoint_position = true;
    cp.checkpoint_latitude_deg = 13.73 + step * 0.001;
    cp.checkpoint_longitude_deg = 100.78 + step * 0.001;
    cp.checkpoint_altitude_m = 15.0;

    auto updated = manager.update_checkpoint(
      wp->get_working_plan_id(), cp, "WAYPOINT_SETTLED");
    ASSERT_TRUE(updated.has_value());

    EXPECT_EQ(updated->get_checkpoint().completed_waypoints, step);
    EXPECT_EQ(updated->get_checkpoint().next_source_index, step);
    EXPECT_FLOAT_EQ(updated->get_checkpoint().progress_percent, (static_cast<float>(step) / 10.0f) * 100.0f);
    EXPECT_EQ(updated->get_checkpoint().checkpoint_reason, "WAYPOINT_SETTLED");

    if (step < 10) {
      EXPECT_EQ(updated->get_state(), domain::WorkingPlanState::SEARCHING);
    } else {
      EXPECT_EQ(updated->get_state(), domain::WorkingPlanState::COMPLETE);
    }
  }
}

// Property 5.3: Resume never silently starts from waypoint 0 when checkpointed
TEST_F(WorkingPlanPropertyTest, ResumeRouteRespectsCheckpoint)
{
  runtime::PlanManager manager;
  auto route = generate_random_route(8);
  std::string json = domain::PlanPrinter::print(route);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  auto art = manager.upload_artifact("resume_test.plan", bytes, 0);
  ASSERT_TRUE(art.has_value());

  auto wp = manager.create_or_select_working_plan(
    art->artifact_id, "kmitl_airfield", "default_scenario", 0);
  ASSERT_TRUE(wp.has_value());

  // Interrupted at waypoint index 4 with a known global position
  double interrupted_lat = 13.7315;
  double interrupted_lon = 100.7885;
  double interrupted_alt = 14.5;

  domain::SearchCheckpointData cp;
  cp.next_source_index = 4;
  cp.completed_waypoints = 4;
  cp.total_waypoints = 8;
  cp.has_checkpoint_position = true;
  cp.checkpoint_latitude_deg = interrupted_lat;
  cp.checkpoint_longitude_deg = interrupted_lon;
  cp.checkpoint_altitude_m = interrupted_alt;

  auto updated = manager.update_checkpoint(
    wp->get_working_plan_id(), cp, "INTERRUPTED_TARGET_ACQUISITION");
  ASSERT_TRUE(updated.has_value());

  auto resume_route = manager.route_for_search(wp->get_working_plan_id());
  ASSERT_TRUE(resume_route.has_value());

  // Resume route must begin at the interrupted entry point
  ASSERT_GE(resume_route->waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(resume_route->waypoints[0].latitude_deg, interrupted_lat);
  EXPECT_DOUBLE_EQ(resume_route->waypoints[0].longitude_deg, interrupted_lon);
  EXPECT_DOUBLE_EQ(resume_route->waypoints[0].altitude_m, interrupted_alt);

  // Subsequent waypoints must be remaining waypoints from index 4..7 (4 waypoints)
  EXPECT_EQ(resume_route->waypoints.size(), 1U + (8U - 4U));  // 1 entry + 4 remaining
  for (size_t i = 1; i < resume_route->waypoints.size(); ++i) {
    size_t source_idx = 4 + (i - 1);
    EXPECT_DOUBLE_EQ(resume_route->waypoints[i].latitude_deg, route.waypoints[source_idx].latitude_deg);
    EXPECT_DOUBLE_EQ(resume_route->waypoints[i].longitude_deg, route.waypoints[source_idx].longitude_deg);
  }
}

// Property 5.4: Reset increments generation, clears checkpoint, resets progress to 0%, preserves source hash
TEST_F(WorkingPlanPropertyTest, ResetBehaviorAndImmutability)
{
  runtime::PlanManager manager;
  auto route = generate_random_route(10);
  std::string json = domain::PlanPrinter::print(route);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  auto art = manager.upload_artifact("reset_test.plan", bytes, 0);
  ASSERT_TRUE(art.has_value());

  auto wp = manager.create_or_select_working_plan(
    art->artifact_id, "kmitl_airfield", "default_scenario", 0);
  ASSERT_TRUE(wp.has_value());

  // Advance checkpoint to 70%
  domain::SearchCheckpointData cp;
  cp.next_source_index = 7;
  cp.completed_waypoints = 7;
  cp.total_waypoints = 10;
  cp.has_checkpoint_position = true;
  cp.checkpoint_latitude_deg = 13.735;
  cp.checkpoint_longitude_deg = 100.785;
  cp.checkpoint_altitude_m = 15.0;
  manager.update_checkpoint(wp->get_working_plan_id(), cp, "TEST_ADVANCE");

  // Reset with invalid confirmation token should fail
  std::string error;
  auto fail_reset = manager.reset_working_plan(
    wp->get_working_plan_id(), 1, "WRONG_TOKEN", &error);
  EXPECT_FALSE(fail_reset.has_value());

  // Reset with valid confirmation "CONFIRM"
  auto reset_wp = manager.reset_working_plan(
    wp->get_working_plan_id(), 1, "CONFIRM", &error);
  ASSERT_TRUE(reset_wp.has_value()) << "Reset failed: " << error;

  // Generation must increment from 1 to 2
  EXPECT_EQ(reset_wp->get_generation(), 2U);

  // Checkpoint must be cleared to 0% and no entry point
  const auto & reset_cp = reset_wp->get_checkpoint();
  EXPECT_EQ(reset_cp.generation, 2U);
  EXPECT_EQ(reset_cp.next_source_index, 0U);
  EXPECT_FALSE(reset_cp.has_checkpoint_position);
  EXPECT_EQ(reset_cp.completed_waypoints, 0U);
  EXPECT_FLOAT_EQ(reset_cp.progress_percent, 0.0f);
  EXPECT_EQ(reset_cp.checkpoint_reason, "RESET");

  // Source artifact hash and route hash must be strictly preserved
  EXPECT_EQ(reset_wp->get_source_artifact_sha256(), art->sha256);
  EXPECT_EQ(reset_wp->get_canonical_route_sha256(), art->route.canonical_route_sha256);

  // Manual plan artifact must still be unchanged
  auto fetched_art = manager.get_artifact(art->artifact_id);
  ASSERT_TRUE(fetched_art.has_value());
  EXPECT_EQ(fetched_art->sha256, art->sha256);
}
