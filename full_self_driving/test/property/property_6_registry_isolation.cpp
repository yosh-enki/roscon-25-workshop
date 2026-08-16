#include <gtest/gtest.h>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "registry/pad_registry.hpp"

using namespace full_self_driving;

class RegistryIsolationPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(42);
  }

  std::mt19937 rng_;

  std::string random_choice(const std::vector<std::string> & options)
  {
    std::uniform_int_distribution<size_t> dist(0, options.size() - 1);
    return options[dist(rng_)];
  }

  uint32_t random_id(uint32_t max_val = 100)
  {
    std::uniform_int_distribution<uint32_t> dist(0, max_val);
    return dist(rng_);
  }

  double random_double(double min_val, double max_val)
  {
    std::uniform_real_distribution<double> dist(min_val, max_val);
    return dist(rng_);
  }
};

// Property 6.1: Lookup Isolation
// A record stored under (map, scenario, ns, dict, id) is retrievable ONLY with exact 5-tuple match.
TEST_F(RegistryIsolationPropertyTest, Property6_LookupIsolation)
{
  const std::vector<std::string> maps = {"kmitl_airfield", "desert_base", "city_quad", "forest_sector"};
  const std::vector<std::string> scenarios = {"scenario_alpha", "scenario_beta", "scenario_gamma"};
  const std::vector<std::string> namespaces = {"aavc2026", "sar_search", "cargo_delivery"};
  const std::vector<std::string> dicts = {"DICT_4X4_50", "DICT_4X4_250", "DICT_5X5_50"};
  const std::vector<uint32_t> ids = {0, 1, 7, 15, 42};

  registry::PadRegistry registry;

  struct RecordEntry
  {
    std::string map;
    std::string scenario;
    std::string ns;
    std::string dict;
    uint32_t id;
    double lat;
    double lon;
    double alt;
  };

  std::vector<RecordEntry> inserted_records;

  // Insert 50 distinct random records
  for (int i = 0; i < 50; ++i) {
    RecordEntry entry;
    entry.map = random_choice(maps);
    entry.scenario = random_choice(scenarios);
    entry.ns = random_choice(namespaces);
    entry.dict = random_choice(dicts);
    entry.id = ids[i % ids.size()];
    entry.lat = random_double(13.0, 14.0);
    entry.lon = random_double(100.0, 101.0);
    entry.alt = random_double(10.0, 50.0);

    msg::PadRecord rec;
    rec.map_id = entry.map;
    rec.scenario_id = entry.scenario;
    rec.identity.marker_id = entry.id;
    rec.identity.dictionary = entry.dict;
    rec.identity.target_namespace = entry.ns;
    rec.latitude_deg = entry.lat;
    rec.longitude_deg = entry.lon;
    rec.altitude_m = entry.alt;
    rec.quality = 0.95f;
    rec.uncertainty_m = 0.05;
    rec.calibration_sha256 = "calib_hash_valid";

    registry.insert_record_for_test(rec);
    inserted_records.push_back(entry);
  }

  // Verify all inserted records can be looked up with exact keys
  for (const auto & entry : inserted_records) {
    domain::TargetIdentity id(entry.id, entry.dict, entry.ns);
    auto found = registry.lookup(id, entry.map, entry.scenario);
    ASSERT_TRUE(found.has_value())
      << "Failed to lookup exact key: map=" << entry.map << ", scenario=" << entry.scenario
      << ", ns=" << entry.ns << ", dict=" << entry.dict << ", id=" << entry.id;
    EXPECT_EQ(found->map_id, entry.map);
    EXPECT_EQ(found->scenario_id, entry.scenario);
    EXPECT_EQ(found->identity.marker_id, entry.id);
    EXPECT_EQ(found->identity.dictionary, entry.dict);
    EXPECT_EQ(found->identity.target_namespace, entry.ns);
  }

  // Verify cross-scope perturbation ALWAYS fails lookup
  for (const auto & entry : inserted_records) {
    domain::TargetIdentity id(entry.id, entry.dict, entry.ns);

    // Perturb map
    for (const auto & other_map : maps) {
      if (other_map != entry.map) {
        // If an entry happened to be inserted in other_map with same scenario, ns, dict, id, it might exist.
        // Otherwise, it must not.
        bool expected_in_other = false;
        for (const auto & other_entry : inserted_records) {
          if (other_entry.map == other_map && other_entry.scenario == entry.scenario &&
              other_entry.ns == entry.ns && other_entry.dict == entry.dict && other_entry.id == entry.id)
          {
            expected_in_other = true;
            break;
          }
        }
        auto res = registry.lookup(id, other_map, entry.scenario);
        EXPECT_EQ(res.has_value(), expected_in_other);
      }
    }

    // Perturb scenario
    for (const auto & other_scenario : scenarios) {
      if (other_scenario != entry.scenario) {
        bool expected_in_other = false;
        for (const auto & other_entry : inserted_records) {
          if (other_entry.map == entry.map && other_entry.scenario == other_scenario &&
              other_entry.ns == entry.ns && other_entry.dict == entry.dict && other_entry.id == entry.id)
          {
            expected_in_other = true;
            break;
          }
        }
        auto res = registry.lookup(id, entry.map, other_scenario);
        EXPECT_EQ(res.has_value(), expected_in_other);
      }
    }

    // Perturb marker_id with non-existent ID
    domain::TargetIdentity non_existent_id(9999, entry.dict, entry.ns);
    auto res_bad_id = registry.lookup(non_existent_id, entry.map, entry.scenario);
    EXPECT_FALSE(res_bad_id.has_value());

    // Perturb dictionary
    domain::TargetIdentity bad_dict_id(entry.id, "NON_EXISTENT_DICT", entry.ns);
    auto res_bad_dict = registry.lookup(bad_dict_id, entry.map, entry.scenario);
    EXPECT_FALSE(res_bad_dict.has_value());

    // Perturb namespace
    domain::TargetIdentity bad_ns_id(entry.id, entry.dict, "NON_EXISTENT_NS");
    auto res_bad_ns = registry.lookup(bad_ns_id, entry.map, entry.scenario);
    EXPECT_FALSE(res_bad_ns.has_value());
  }
}

// Property 6.2: Snapshot Isolation
// Snapshots contain ONLY records matching the requested (map, scenario) scope.
TEST_F(RegistryIsolationPropertyTest, Property6_SnapshotIsolation)
{
  registry::PadRegistry registry;

  const std::string map_a = "kmitl_airfield";
  const std::string map_b = "desert_base";
  const std::string scenario_1 = "scenario_alpha";
  const std::string scenario_2 = "scenario_beta";

  // Insert records in 4 combinations
  for (uint32_t id = 0; id < 5; ++id) {
    msg::PadRecord r1;
    r1.map_id = map_a; r1.scenario_id = scenario_1;
    r1.identity.marker_id = id; r1.identity.dictionary = "DICT_4X4_50"; r1.identity.target_namespace = "ns1";
    registry.insert_record_for_test(r1);

    msg::PadRecord r2;
    r2.map_id = map_a; r2.scenario_id = scenario_2;
    r2.identity.marker_id = id + 10; r2.identity.dictionary = "DICT_4X4_50"; r2.identity.target_namespace = "ns1";
    registry.insert_record_for_test(r2);

    msg::PadRecord r3;
    r3.map_id = map_b; r3.scenario_id = scenario_1;
    r3.identity.marker_id = id + 20; r3.identity.dictionary = "DICT_4X4_50"; r3.identity.target_namespace = "ns1";
    registry.insert_record_for_test(r3);
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  uint64_t monotonic_ns = 1000000000;

  auto snap_a1 = registry.get_snapshot(map_a, scenario_1, stamp, monotonic_ns);
  EXPECT_EQ(snap_a1.map_id, map_a);
  EXPECT_EQ(snap_a1.scenario_id, scenario_1);
  EXPECT_EQ(snap_a1.records.size(), 5u);
  for (const auto & rec : snap_a1.records) {
    EXPECT_EQ(rec.map_id, map_a);
    EXPECT_EQ(rec.scenario_id, scenario_1);
    EXPECT_LT(rec.identity.marker_id, 10u);
  }

  auto snap_a2 = registry.get_snapshot(map_a, scenario_2, stamp, monotonic_ns);
  EXPECT_EQ(snap_a2.records.size(), 5u);
  for (const auto & rec : snap_a2.records) {
    EXPECT_EQ(rec.map_id, map_a);
    EXPECT_EQ(rec.scenario_id, scenario_2);
    EXPECT_GE(rec.identity.marker_id, 10u);
    EXPECT_LT(rec.identity.marker_id, 20u);
  }

  auto snap_b1 = registry.get_snapshot(map_b, scenario_1, stamp, monotonic_ns);
  EXPECT_EQ(snap_b1.records.size(), 5u);
  for (const auto & rec : snap_b1.records) {
    EXPECT_EQ(rec.map_id, map_b);
    EXPECT_EQ(rec.scenario_id, scenario_1);
    EXPECT_GE(rec.identity.marker_id, 20u);
  }

  auto snap_b2 = registry.get_snapshot(map_b, scenario_2, stamp, monotonic_ns);
  EXPECT_EQ(snap_b2.records.size(), 0u);
}

// Property 6.3: Clear Preconditions, Backup, and Scope Isolation
// Clear is allowed ONLY when disarmed, with exact expected revision, confirmation, and affects ONLY the target scope.
TEST_F(RegistryIsolationPropertyTest, Property6_ClearPreconditionsAndScopeIsolation)
{
  registry::PadRegistry registry;

  const std::string map_a = "kmitl_airfield";
  const std::string map_b = "desert_base";
  const std::string scenario_1 = "scenario_alpha";

  for (uint32_t id = 0; id < 4; ++id) {
    msg::PadRecord r1;
    r1.map_id = map_a; r1.scenario_id = scenario_1;
    r1.identity.marker_id = id; r1.identity.dictionary = "DICT_4X4_50"; r1.identity.target_namespace = "ns1";
    registry.insert_record_for_test(r1);

    msg::PadRecord r2;
    r2.map_id = map_b; r2.scenario_id = scenario_1;
    r2.identity.marker_id = id + 10; r2.identity.dictionary = "DICT_4X4_50"; r2.identity.target_namespace = "ns1";
    registry.insert_record_for_test(r2);
  }

  uint64_t current_rev = registry.get_revision();
  EXPECT_EQ(registry.size(map_a, scenario_1), 4u);
  EXPECT_EQ(registry.size(map_b, scenario_1), 4u);

  // 1. Attempt clear when armed -> MUST FAIL
  auto res_armed = registry.clear(map_a, scenario_1, current_rev, "CONFIRM_CLEAR", false);
  EXPECT_FALSE(res_armed.success);
  EXPECT_EQ(registry.size(map_a, scenario_1), 4u);
  EXPECT_EQ(registry.get_revision(), current_rev);

  // 2. Attempt clear with wrong revision -> MUST FAIL
  auto res_bad_rev = registry.clear(map_a, scenario_1, current_rev + 99, "CONFIRM_CLEAR", true);
  EXPECT_FALSE(res_bad_rev.success);
  EXPECT_EQ(registry.size(map_a, scenario_1), 4u);
  EXPECT_EQ(registry.get_revision(), current_rev);

  // 3. Attempt clear with wrong confirmation token -> MUST FAIL
  auto res_bad_conf = registry.clear(map_a, scenario_1, current_rev, "WRONG_TOKEN", true);
  EXPECT_FALSE(res_bad_conf.success);
  EXPECT_EQ(registry.size(map_a, scenario_1), 4u);
  EXPECT_EQ(registry.get_revision(), current_rev);

  // 4. Successful clear: disarmed, matching revision, valid confirmation
  auto res_ok = registry.clear(map_a, scenario_1, current_rev, "CONFIRM_CLEAR", true);
  EXPECT_TRUE(res_ok.success);
  EXPECT_GT(registry.get_revision(), current_rev);

  // Map A Scope 1 is now empty
  EXPECT_EQ(registry.size(map_a, scenario_1), 0u);

  // Map B Scope 1 is completely untouched!
  EXPECT_EQ(registry.size(map_b, scenario_1), 4u);
  for (uint32_t id = 0; id < 4; ++id) {
    domain::TargetIdentity target_id(id + 10, "DICT_4X4_50", "ns1");
    auto found = registry.lookup(target_id, map_b, scenario_1);
    EXPECT_TRUE(found.has_value());
  }
}
