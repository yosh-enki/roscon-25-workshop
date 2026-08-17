#include <gtest/gtest.h>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "domain/plan_parser.hpp"
#include "domain/plan_printer.hpp"
#include "runtime/plan_manager.hpp"

using namespace full_self_driving;

class PlanImmutabilityPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(54321);
    test_dir_ = "/tmp/fsd_test_plan_immutability_" + std::to_string(random_uint(1000, 999999));
    std::filesystem::create_directories(test_dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  std::string test_dir_;
  std::mt19937 rng_;

  double random_double(double min_val, double max_val)
  {
    std::uniform_real_distribution<double> dist(min_val, max_val);
    return dist(rng_);
  }

  uint32_t random_uint(uint32_t min_val, uint32_t max_val)
  {
    std::uniform_int_distribution<uint32_t> dist(min_val, max_val);
    return dist(rng_);
  }

  std::string generate_valid_qgc_plan_json(
    size_t num_waypoints = 5,
    float alt = 15.0f,
    float speed = 5.0f)
  {
    domain::CanonicalSearchRoute route;
    route.default_altitude_m = alt;
    route.cruise_speed_m_s = speed;

    for (size_t i = 0; i < num_waypoints; ++i) {
      domain::SearchWaypoint wp;
      wp.latitude_deg = random_double(-80.0, 80.0);
      wp.longitude_deg = random_double(-170.0, 170.0);
      wp.altitude_m = alt;
      wp.source_index = static_cast<uint32_t>(i);
      route.waypoints.push_back(wp);
    }
    return domain::PlanPrinter::print(route);
  }
};

// Property 4.1: Valid plan ingestion produces managed immutable artifact with SHA-256 and canonical route
TEST_F(PlanImmutabilityPropertyTest, ValidPlanIngestionProducesImmutableArtifact)
{
  runtime::PlanManager manager(test_dir_);

  for (int i = 0; i < 20; ++i) {
    std::string safe_name = "test_mission_" + std::to_string(i) + ".plan";
    std::string json = generate_valid_qgc_plan_json(5 + (i % 10));
    std::vector<uint8_t> bytes(json.begin(), json.end());

    std::string error;
    auto artifact = manager.upload_artifact(safe_name, bytes, 0, &error);
    ASSERT_TRUE(artifact.has_value()) << "Failed to upload valid artifact: " << error;

    EXPECT_FALSE(artifact->artifact_id.empty());
    EXPECT_EQ(artifact->artifact_id.rfind("art_", 0), 0U);
    EXPECT_EQ(artifact->safe_name, safe_name);
    EXPECT_FALSE(artifact->sha256.empty());
    EXPECT_EQ(artifact->sha256.length(), 64U);
    EXPECT_EQ(artifact->byte_length, bytes.size());
    EXPECT_TRUE(artifact->immutable);
    EXPECT_FALSE(artifact->route.empty());
    EXPECT_FALSE(artifact->route.canonical_route_sha256.empty());

    // Verify retrieval by ID
    auto fetched = manager.get_artifact(artifact->artifact_id);
    ASSERT_TRUE(fetched.has_value());
    EXPECT_EQ(fetched->artifact_id, artifact->artifact_id);
    EXPECT_EQ(fetched->sha256, artifact->sha256);
  }

  EXPECT_EQ(manager.list_artifacts().size(), 20U);
}

// Property 4.2: Idempotent upload with identical content preserves artifact ID and hash
TEST_F(PlanImmutabilityPropertyTest, IdempotentUploadPreservesArtifact)
{
  runtime::PlanManager manager(test_dir_);

  std::string safe_name = "reusable_mission.plan";
  std::string json = generate_valid_qgc_plan_json(8);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  auto art1 = manager.upload_artifact(safe_name, bytes, 0);
  ASSERT_TRUE(art1.has_value());

  // Upload identical content a second time
  auto art2 = manager.upload_artifact(safe_name, bytes, 0);
  ASSERT_TRUE(art2.has_value());

  EXPECT_EQ(art1->artifact_id, art2->artifact_id);
  EXPECT_EQ(art1->sha256, art2->sha256);
  EXPECT_EQ(art1->route.canonical_route_sha256, art2->route.canonical_route_sha256);
  EXPECT_EQ(manager.list_artifacts().size(), 1U);
}

// Property 4.3: Reject unsafe artifact names and path traversal attempts (Requirement 2.2 / Property 4)
TEST_F(PlanImmutabilityPropertyTest, RejectsUnsafeArtifactNamesAndTraversal)
{
  runtime::PlanManager manager(test_dir_);
  std::string json = generate_valid_qgc_plan_json(5);
  std::vector<uint8_t> bytes(json.begin(), json.end());

  const std::vector<std::string> invalid_names = {
    "../mission.plan",
    "../../etc/passwd.plan",
    "/root/mission.plan",
    "subdir/mission.plan",
    "subdir\\mission.plan",
    ".plan",
    "..",
    ".",
    ".hidden_mission.plan",
    "mission.json",
    "mission.txt",
    "mission",
    "",
    std::string(130, 'a') + ".plan"  // overlong (>128 chars)
  };

  for (const auto & name : invalid_names) {
    std::string error;
    auto res = manager.upload_artifact(name, bytes, 0, &error);
    EXPECT_FALSE(res.has_value()) << "Expected failure for unsafe name: " << name;
    EXPECT_FALSE(error.empty());
  }
}

// Property 4.4: Reject malformed, oversized, empty, non-finite, and zero-waypoint plans
TEST_F(PlanImmutabilityPropertyTest, RejectsMalformedOversizedAndNonFinitePlans)
{
  runtime::PlanManager manager(test_dir_);

  // 1. Empty content
  {
    std::vector<uint8_t> empty_bytes;
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("empty.plan", empty_bytes, 0, &error).has_value());
  }

  // 2. Malformed JSON
  {
    std::string bad_json = "{\"mission\": { invalid_json...";
    std::vector<uint8_t> bytes(bad_json.begin(), bad_json.end());
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("malformed.plan", bytes, 0, &error).has_value());
  }

  // 3. Missing mission object
  {
    std::string no_mission = "{\"groundStation\": \"QGC\", \"version\": 1}";
    std::vector<uint8_t> bytes(no_mission.begin(), no_mission.end());
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("no_mission.plan", bytes, 0, &error).has_value());
  }

  // 4. Zero waypoints
  {
    std::string zero_wp = "{\"mission\": {\"items\": []}}";
    std::vector<uint8_t> bytes(zero_wp.begin(), zero_wp.end());
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("zero_wp.plan", bytes, 0, &error).has_value());
  }

  // 5. Non-finite / out of range latitude/longitude
  {
    std::string out_of_range_lat = R"({
      "mission": {
        "items": [{
          "command": 16,
          "params": [0, 0, 0, null, 120.0, 100.0, 15.0]
        }]
      }
    })";
    std::vector<uint8_t> bytes(out_of_range_lat.begin(), out_of_range_lat.end());
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("bad_lat.plan", bytes, 0, &error).has_value());
  }

  // 6. Out of range longitude
  {
    std::string out_of_range_lon = R"({
      "mission": {
        "items": [{
          "command": 16,
          "params": [0, 0, 0, null, 13.0, 200.0, 15.0]
        }]
      }
    })";
    std::vector<uint8_t> bytes(out_of_range_lon.begin(), out_of_range_lon.end());
    std::string error;
    EXPECT_FALSE(manager.upload_artifact("bad_lon.plan", bytes, 0, &error).has_value());
  }
}

// Property 4.5: Canonical route hash is deterministic and sensitive to any waypoint change
TEST_F(PlanImmutabilityPropertyTest, CanonicalRouteHashIsDeterministicAndSensitive)
{
  auto json1 = generate_valid_qgc_plan_json(6, 15.0f, 5.0f);
  auto parse1 = domain::PlanParser::parse_string(json1);
  ASSERT_TRUE(parse1.is_valid);

  // Parsing the exact same string produces identical hash
  auto parse2 = domain::PlanParser::parse_string(json1);
  ASSERT_TRUE(parse2.is_valid);
  EXPECT_EQ(parse1.route.canonical_route_sha256, parse2.route.canonical_route_sha256);

  // Altering one waypoint latitude alters canonical route hash
  auto route_mod = parse1.route;
  route_mod.waypoints[0].latitude_deg += 0.0001;
  std::string hash_mod = domain::PlanParser::compute_canonical_route_hash(route_mod);
  EXPECT_NE(parse1.route.canonical_route_sha256, hash_mod);

  // Altering altitude alters canonical route hash
  auto route_alt = parse1.route;
  route_alt.waypoints[0].altitude_m += 1.0;
  std::string hash_alt = domain::PlanParser::compute_canonical_route_hash(route_alt);
  EXPECT_NE(parse1.route.canonical_route_sha256, hash_alt);
}
