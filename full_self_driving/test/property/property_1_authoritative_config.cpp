#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <regex>
#include <string>
#include <vector>

#include "domain/engineering_config.hpp"

using namespace full_self_driving;

class AuthoritativeConfigPropertyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rng_.seed(12345);
  }

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
};

// Property 1.1: Valid configurations pass validation
TEST_F(AuthoritativeConfigPropertyTest, Property1_ValidConfigPassesValidation)
{
  for (int i = 0; i < 50; ++i) {
    domain::EngineeringConfig config = domain::EngineeringConfig::create_default_simulation_config();
    config.deployment_id = "deployment_" + std::to_string(i);
    config.safety.max_altitude_m = random_double(10.0, 100.0);
    config.safety.min_battery_percentage = random_double(5.0, 40.0);
    config.safety.target_loss_timeout_s = random_double(0.5, 10.0);

    config.routes.approach_altitude_m = random_double(2.0, 10.0);
    config.routes.search_altitude_m = config.routes.approach_altitude_m + random_double(1.0, 20.0);
    config.routes.transit_in_speed_m_s = random_double(1.0, 15.0);
    config.routes.transit_out_speed_m_s = random_double(1.0, 15.0);
    config.routes.max_horizontal_velocity_m_s = random_double(2.0, 20.0);
    config.routes.landing_descent_rate_m_s = random_double(0.1, 2.0);

    auto result = config.validate();
    EXPECT_TRUE(result.is_valid) << "Failed validation for valid config iteration " << i;
    EXPECT_TRUE(result.violations.empty());
  }
}

// Property 1.2: Bounding and relationship violations are rejected
TEST_F(AuthoritativeConfigPropertyTest, Property1_BoundingViolationsRejected)
{
  // 1. Invalid schema version
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.schema_version = "2.0.0";
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 2. Empty deployment ID
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.deployment_id = "";
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 3. Overlong deployment ID (>64 chars)
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.deployment_id = std::string(65, 'a');
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 4. Invalid profile
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.profile = "invalid_profile";
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 5. Negative or zero safety parameters
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.safety.max_altitude_m = -10.0;
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.safety.max_altitude_m = 0.0;
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.safety.min_battery_percentage = -1.0;
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.safety.min_battery_percentage = 101.0;
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.safety.target_loss_timeout_s = 0.0;
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 6. Invalid speeds and descent rates
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.routes.transit_in_speed_m_s = -1.0;
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.routes.landing_descent_rate_m_s = 0.0;
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 7. Relationship violation: search_altitude < approach_altitude
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.routes.search_altitude_m = 4.0;
    cfg.routes.approach_altitude_m = 10.0;
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 8. Empty adapters
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.adapters.px4_transport = "";
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.adapters.camera_adapter = "";
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.adapters.payload_adapter = "";
    EXPECT_FALSE(cfg.validate().is_valid);
  }

  // 9. Target constraints violations
  {
    auto cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.target_constraints.marker_id_min = 500;
    cfg.target_constraints.marker_id_max = 100;  // max < min
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.target_constraints.allowed_dictionaries.clear();
    EXPECT_FALSE(cfg.validate().is_valid);

    cfg = domain::EngineeringConfig::create_default_simulation_config();
    cfg.target_constraints.allowed_namespaces.clear();
    EXPECT_FALSE(cfg.validate().is_valid);
  }
}

// Property 1.3: Canonical hash format and determinism
TEST_F(AuthoritativeConfigPropertyTest, Property1_CanonicalHashDeterminismAndFormat)
{
  auto cfg = domain::EngineeringConfig::create_default_simulation_config();
  std::string hash1 = cfg.compute_canonical_hash();
  std::string hash2 = cfg.compute_canonical_hash();

  EXPECT_EQ(hash1, hash2);
  EXPECT_EQ(hash1.size(), 64u);

  // Must be lowercase hex
  std::regex hex_regex("^[0-9a-f]{64}$");
  EXPECT_TRUE(std::regex_match(hash1, hex_regex));
}

// Property 1.4: Any field change alters the canonical hash
TEST_F(AuthoritativeConfigPropertyTest, Property1_HashSensitivityToMutations)
{
  auto base_cfg = domain::EngineeringConfig::create_default_simulation_config();
  std::string base_hash = base_cfg.compute_canonical_hash();

  // Perturb deployment_id
  {
    auto cfg = base_cfg;
    cfg.deployment_id = "modified_deployment";
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb revision
  {
    auto cfg = base_cfg;
    cfg.engineering_config_revision = 2;
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb speed
  {
    auto cfg = base_cfg;
    cfg.routes.transit_in_speed_m_s = base_cfg.routes.transit_in_speed_m_s + 0.1;
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb altitude
  {
    auto cfg = base_cfg;
    cfg.routes.search_altitude_m = base_cfg.routes.search_altitude_m + 0.5;
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb safety
  {
    auto cfg = base_cfg;
    cfg.safety.max_altitude_m = base_cfg.safety.max_altitude_m + 1.0;
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb dictionary set
  {
    auto cfg = base_cfg;
    cfg.target_constraints.allowed_dictionaries.push_back("DICT_6X6_250");
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }

  // Perturb adapters
  {
    auto cfg = base_cfg;
    cfg.adapters.px4_transport = "px4_serial_transport";
    EXPECT_NE(cfg.compute_canonical_hash(), base_hash);
  }
}

// Property 1.5: YAML loader round-trip and validation
TEST_F(AuthoritativeConfigPropertyTest, Property1_YamlLoaderCorrectness)
{
  std::string yaml_text = R"(
schema_version: "1.0.0"
deployment_id: "test_kmitl_deployment"
profile: "simulation"
engineering_config_revision: 42

simulation:
  world: "kmitl_airfield"
  headless: true
  use_sim_time: true

safety:
  max_altitude_m: 25.0
  min_battery_percentage: 15.0
  target_loss_timeout_s: 3.0
  emergency_stop_enabled: true

routes:
  transit_in_speed_m_s: 4.5
  transit_out_speed_m_s: 4.0
  search_altitude_m: 12.0
  approach_altitude_m: 6.0
  max_horizontal_velocity_m_s: 6.0
  landing_descent_rate_m_s: 0.8

adapters:
  px4_transport: "px4_sitl_uxrce_dds"
  camera_adapter: "ros_gz_image_bridge"
  payload_adapter: "simulation_payload_stub"

target_constraints:
  marker_id_min: 1
  marker_id_max: 50
  allowed_dictionaries:
    - "DICT_4X4_50"
    - "DICT_5X5_50"
  allowed_namespaces:
    - "aavc2026"
)";

  auto config = domain::EngineeringConfig::from_yaml_string(yaml_text);
  auto validation = config.validate();
  EXPECT_TRUE(validation.is_valid);
  EXPECT_EQ(config.schema_version, "1.0.0");
  EXPECT_EQ(config.deployment_id, "test_kmitl_deployment");
  EXPECT_EQ(config.engineering_config_revision, 42u);
  EXPECT_DOUBLE_EQ(config.routes.transit_in_speed_m_s, 4.5);
  EXPECT_DOUBLE_EQ(config.safety.max_altitude_m, 25.0);
  EXPECT_EQ(config.target_constraints.marker_id_min, 1u);
  EXPECT_EQ(config.target_constraints.marker_id_max, 50u);
  EXPECT_EQ(config.target_constraints.allowed_dictionaries.size(), 2u);

  std::string hash = config.compute_canonical_hash();
  EXPECT_EQ(hash.size(), 64u);
}
