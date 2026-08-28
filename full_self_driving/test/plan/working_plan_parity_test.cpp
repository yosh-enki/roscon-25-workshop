#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"
#include "domain/working_plan.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class WorkingPlanParityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Find the fixture file
    std::string share_dir;
    try {
      share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
    } catch (...) {
      share_dir = "";
    }

    std::vector<std::string> candidates = {
      share_dir + "/test/fixtures/plans/aavc2026_mission.plan",
      share_dir + "/share/full_self_driving/test/fixtures/plans/aavc2026_mission.plan",
      "test/fixtures/plans/aavc2026_mission.plan"
    };

    for (const auto & path : candidates) {
      if (!path.empty() && std::filesystem::is_regular_file(path)) {
        fixture_path_ = path;
        break;
      }
    }
  }

  std::string fixture_path_;
};

// Test 1: Validate aavc2026_mission.plan extraction against prototype ground truth
TEST_F(WorkingPlanParityTest, PrototypeAavc2026MissionParity)
{
  ASSERT_FALSE(fixture_path_.empty()) << "Fixture aavc2026_mission.plan not found";

  auto parse_result = domain::PlanParser::parse_file(fixture_path_);
  ASSERT_TRUE(parse_result.is_valid) << "Failed to parse fixture: " << parse_result.error_message;

  // Expected values from prototype Search baseline
  EXPECT_EQ(parse_result.route.waypoints.size(), 10U);
  EXPECT_FLOAT_EQ(parse_result.route.default_altitude_m, 15.0f);
  EXPECT_FLOAT_EQ(parse_result.route.cruise_speed_m_s, 5.0f);

  // First waypoint: (13.731328451896626, 100.78990948056993, 15.0)
  EXPECT_NEAR(parse_result.route.waypoints[0].latitude_deg, 13.73132845, 1e-6);
  EXPECT_NEAR(parse_result.route.waypoints[0].longitude_deg, 100.78990948, 1e-6);
  EXPECT_DOUBLE_EQ(parse_result.route.waypoints[0].altitude_m, 15.0);

  // Last waypoint (index 9): (13.7307894714774, 100.78783793887287, 15.0)
  EXPECT_NEAR(parse_result.route.waypoints[9].latitude_deg, 13.73078947, 1e-6);
  EXPECT_NEAR(parse_result.route.waypoints[9].longitude_deg, 100.78783793, 1e-6);
  EXPECT_DOUBLE_EQ(parse_result.route.waypoints[9].altitude_m, 15.0);
}

// Test 2: Verify SearchPlanner algorithm parity (Route generation, resume entry point, and holding)
TEST_F(WorkingPlanParityTest, SearchPlannerAlgorithmParity)
{
  ASSERT_FALSE(fixture_path_.empty());
  auto parse_result = domain::PlanParser::parse_file(fixture_path_);
  ASSERT_TRUE(parse_result.is_valid);

  domain::WorkingPlan wp(
    "wp_test_aavc2026",
    "art_aavc2026",
    "kmitl_airfield",
    "default_scenario",
    parse_result.raw_content_sha256,
    parse_result.route);

  // 1. Initial route has all 10 waypoints
  auto initial_search_route = wp.route_for_search();
  EXPECT_EQ(initial_search_route.waypoints.size(), 10U);

  // 2. Interrupted at waypoint 4 -> resume has entry point + remaining waypoints (4..9 = 6 waypoints) -> 7 total
  domain::SearchCheckpointData cp;
  cp.next_source_index = 4;
  cp.completed_waypoints = 4;
  cp.total_waypoints = 10;
  cp.has_checkpoint_position = true;
  cp.checkpoint_latitude_deg = 13.7311;
  cp.checkpoint_longitude_deg = 100.7888;
  cp.checkpoint_altitude_m = 15.0;

  wp.update_checkpoint(cp, "PAUSED_PRECISION_LAND");
  auto resumed_route = wp.route_for_search();
  EXPECT_EQ(resumed_route.waypoints.size(), 7U);  // 1 entry + 6 remaining
  EXPECT_DOUBLE_EQ(resumed_route.waypoints[0].latitude_deg, 13.7311);
  EXPECT_DOUBLE_EQ(resumed_route.waypoints[0].longitude_deg, 100.7888);

  // 3. Completed route (all 10 waypoints done) -> holds at the final waypoint
  cp.next_source_index = 10;
  cp.completed_waypoints = 10;
  cp.has_checkpoint_position = false;
  wp.update_checkpoint(cp, "ALL_WAYPOINTS_COMPLETED");
  EXPECT_EQ(wp.get_state(), domain::WorkingPlanState::COMPLETE);

  auto completed_route = wp.route_for_search();
  ASSERT_EQ(completed_route.waypoints.size(), 1U);
  EXPECT_NEAR(completed_route.waypoints[0].latitude_deg, 13.73078947, 1e-6);
  EXPECT_NEAR(completed_route.waypoints[0].longitude_deg, 100.78783793, 1e-6);

  // 4. Reset -> starts fresh generation with 0% progress and 10 waypoints
  std::string error;
  bool reset_ok = wp.reset(1, "CONFIRM", &error);
  ASSERT_TRUE(reset_ok);
  EXPECT_EQ(wp.get_generation(), 2U);
  EXPECT_EQ(wp.get_checkpoint().next_source_index, 0U);
  EXPECT_FLOAT_EQ(wp.get_checkpoint().progress_percent, 0.0f);
  EXPECT_EQ(wp.route_for_search().waypoints.size(), 10U);
}

// Test 3: Test parsing the original Desktop .plan file directly
TEST_F(WorkingPlanParityTest, DesktopOriginalPlanVerification)
{
  std::string desktop_plan = fixture_path_;

  ASSERT_TRUE(std::filesystem::is_regular_file(desktop_plan))
    << "Desktop plan file not found at " << desktop_plan;

  auto result = domain::PlanParser::parse_file(desktop_plan);
  ASSERT_TRUE(result.is_valid) << "Failed to parse Desktop .plan: " << result.error_message;

  std::cout << "\n========================================================\n";
  std::cout << "  [SUCCESS] Desktop .plan Successfully Parsed!\n";
  std::cout << "========================================================\n";
  std::cout << "  File: " << desktop_plan << "\n";
  std::cout << "  File Size: " << result.byte_length << " bytes\n";
  std::cout << "  Raw SHA-256: " << result.raw_content_sha256 << "\n";
  std::cout << "  Canonical Route SHA-256: " << result.route.canonical_route_sha256 << "\n";
  std::cout << "  Cruise Speed: " << result.route.cruise_speed_m_s << " m/s\n";
  std::cout << "  Default Altitude: " << result.route.default_altitude_m << " m\n";
  std::cout << "  Total Waypoints Extracted: " << result.route.waypoints.size() << "\n\n";

  for (size_t i = 0; i < result.route.waypoints.size(); ++i) {
    const auto & wp = result.route.waypoints[i];
    std::cout << "  WP [" << (i + 1) << "] Lat: " << std::fixed << std::setprecision(7)
              << wp.latitude_deg << " | Lon: " << wp.longitude_deg
              << " | Alt: " << wp.altitude_m << " m (SourceIndex: " << wp.source_index << ")\n";
  }
  std::cout << "========================================================\n";

  EXPECT_GT(result.route.waypoints.size(), 0U);
  EXPECT_FALSE(result.raw_content_sha256.empty());
  EXPECT_FALSE(result.route.canonical_route_sha256.empty());
}

// Test 4: WorkingPlan with Transit In and Transit Out routes
TEST_F(WorkingPlanParityTest, WorkingPlanWithTransitRoutes)
{
  std::string share_dir;
  try {
    share_dir = ament_index_cpp::get_package_share_directory("full_self_driving");
  } catch (...) {}
  std::string kmitl_path = share_dir + "/test/fixtures/plans/kmitl.plan";
  if (!std::filesystem::is_regular_file(kmitl_path)) {
    kmitl_path = "test/fixtures/plans/kmitl.plan";
  }
  if (std::filesystem::is_regular_file(kmitl_path)) {
    runtime::PlanManager pm;
    std::ifstream file(kmitl_path, std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::string err;
    auto art = pm.upload_artifact("kmitl.plan", bytes, 0, &err);
    ASSERT_TRUE(art.has_value()) << "Failed upload: " << err;
    EXPECT_EQ(art->map_name, "kmitl");
    EXPECT_EQ(art->transit_in_waypoints.size(), 3U);
    EXPECT_EQ(art->transit_out_waypoints.size(), 3U);

    auto wp = pm.create_or_select_working_plan(art->artifact_id, "kmitl", "default_scenario", 0, &err);
    ASSERT_TRUE(wp.has_value()) << "Failed working plan: " << err;
    EXPECT_TRUE(wp->has_transit_in_route());
    EXPECT_TRUE(wp->has_transit_out_route());
    EXPECT_EQ(wp->get_transit_in_route().size(), 3U);
    EXPECT_EQ(wp->get_transit_out_route().size(), 3U);
    EXPECT_NEAR(wp->get_transit_in_route()[0].latitude_deg, 13.730322, 1e-6);
    EXPECT_NEAR(wp->get_transit_out_route()[0].latitude_deg, 13.730712, 1e-6);
  }
}


