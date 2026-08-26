#include <gtest/gtest.h>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"

using namespace full_self_driving;

class PlanRoundTripTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(424242);
  }

  std::mt19937 rng_;

  double random_double(double min_val, double max_val)
  {
    std::uniform_real_distribution<double> dist(min_val, max_val);
    return dist(rng_);
  }

  domain::CanonicalSearchRoute generate_route(size_t num_waypoints, float alt, float speed)
  {
    domain::CanonicalSearchRoute route;
    route.default_altitude_m = alt;
    route.cruise_speed_m_s = speed;

    for (size_t i = 0; i < num_waypoints; ++i) {
      domain::SearchWaypoint wp;
      wp.latitude_deg = random_double(13.72, 13.74);
      wp.longitude_deg = random_double(100.78, 100.80);
      wp.altitude_m = alt;
      wp.source_index = static_cast<uint32_t>(i);
      route.waypoints.push_back(wp);
    }
    route.canonical_route_sha256 = domain::PlanParser::compute_canonical_route_hash(route);
    return route;
  }
};

// Test 1: PlanPrinter -> PlanParser round-trip produces identical route and hash
TEST_F(PlanRoundTripTest, PrinterParserRoundTripIdempotency)
{
  for (size_t count : {1, 5, 20, 50}) {
    float alt = 15.0f;
    float speed = 6.5f;
    auto initial_route = generate_route(count, alt, speed);

    // 1. Serialize route to QGC JSON
    std::string printed_json = domain::PlanPrinter::print(initial_route);
    EXPECT_FALSE(printed_json.empty());

    // 2. Parse serialized QGC JSON
    auto parse_result = domain::PlanParser::parse_string(printed_json);
    ASSERT_TRUE(parse_result.is_valid) << "Parse error: " << parse_result.error_message;

    // 3. Verify route attributes
    EXPECT_EQ(parse_result.route.waypoints.size(), initial_route.waypoints.size());
    EXPECT_FLOAT_EQ(parse_result.route.default_altitude_m, initial_route.default_altitude_m);
    EXPECT_FLOAT_EQ(parse_result.route.cruise_speed_m_s, initial_route.cruise_speed_m_s);
    EXPECT_EQ(parse_result.route.canonical_route_sha256, initial_route.canonical_route_sha256);

    for (size_t i = 0; i < count; ++i) {
      EXPECT_DOUBLE_EQ(parse_result.route.waypoints[i].latitude_deg, initial_route.waypoints[i].latitude_deg);
      EXPECT_DOUBLE_EQ(parse_result.route.waypoints[i].longitude_deg, initial_route.waypoints[i].longitude_deg);
      EXPECT_DOUBLE_EQ(parse_result.route.waypoints[i].altitude_m, initial_route.waypoints[i].altitude_m);
      EXPECT_EQ(parse_result.route.waypoints[i].source_index, initial_route.waypoints[i].source_index);
    }

    // 4. Double round-trip (print again -> parse again)
    std::string double_printed = domain::PlanPrinter::print(parse_result.route);
    auto double_parse = domain::PlanParser::parse_string(double_printed);
    ASSERT_TRUE(double_parse.is_valid);
    EXPECT_EQ(double_parse.route.canonical_route_sha256, initial_route.canonical_route_sha256);
  }
}

// Test 2: Round-trip with search planner resume metadata
TEST_F(PlanRoundTripTest, RoundTripWithSearchPlannerMetadata)
{
  auto initial_route = generate_route(15, 20.0f, 5.0f);
  std::array<double, 2> entry_pt = {13.7314, 100.7892};
  size_t next_idx = 7;

  std::string printed_json = domain::PlanPrinter::print(initial_route, entry_pt, next_idx);
  auto parsed = domain::PlanParser::parse_string(printed_json);

  ASSERT_TRUE(parsed.is_valid);
  EXPECT_TRUE(parsed.has_search_planner_metadata);
  ASSERT_TRUE(parsed.entry_point.has_value());
  EXPECT_DOUBLE_EQ((*parsed.entry_point)[0], entry_pt[0]);
  EXPECT_DOUBLE_EQ((*parsed.entry_point)[1], entry_pt[1]);
  EXPECT_EQ(parsed.next_waypoint_index, next_idx);
  EXPECT_EQ(parsed.route.canonical_route_sha256, initial_route.canonical_route_sha256);
}

// Test 3: Plan-centric classified waypoint extraction (Transit In, Search, Transit Out)
TEST_F(PlanRoundTripTest, ClassifiedWaypointsExtraction)
{
  std::string kmitl_path = "/home/yosh/Documents/QGroundControl/Missions/kmitl.plan";
  if (std::filesystem::is_regular_file(kmitl_path)) {
    auto parsed = domain::PlanParser::parse_file(kmitl_path);
    ASSERT_TRUE(parsed.is_valid) << "Parse error: " << parsed.error_message;
    EXPECT_EQ(parsed.map_name, "kmitl");
    EXPECT_EQ(parsed.transit_in_waypoints.size(), 3U);
    EXPECT_EQ(parsed.transit_out_waypoints.size(), 3U);
    EXPECT_GT(parsed.route.waypoints.size(), 0U);

    // Verify first Transit In waypoint
    EXPECT_NEAR(parsed.transit_in_waypoints[0].latitude_deg, 13.730322, 1e-6);
    EXPECT_NEAR(parsed.transit_in_waypoints[0].longitude_deg, 100.787446, 1e-6);

    // Verify first Transit Out waypoint
    EXPECT_NEAR(parsed.transit_out_waypoints[0].latitude_deg, 13.730712, 1e-6);
    EXPECT_NEAR(parsed.transit_out_waypoints[0].longitude_deg, 100.788755, 1e-6);
  }
}

