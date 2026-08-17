#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace full_self_driving::domain
{

struct RoutePolicy
{
  double transit_in_speed_m_s{5.0};
  double transit_out_speed_m_s{3.0};
  double search_altitude_m{10.0};
  double approach_altitude_m{5.0};
  double max_horizontal_velocity_m_s{5.0};
  double landing_descent_rate_m_s{0.5};
  double acceptance_radius_m{4.0};
  double max_yaw_rate_deg_s{45.0};
};

struct SafetyPolicy
{
  double max_altitude_m{30.0};
  double min_battery_percentage{20.0};
  double target_loss_timeout_s{2.0};
  bool emergency_stop_enabled{true};
};

struct SimulationPolicy
{
  std::string world{"kmitl_airfield"};
  bool headless{true};
  bool use_sim_time{true};
};

struct AdaptersConfig
{
  std::string px4_transport{"px4_sitl_uxrce_dds"};
  std::string camera_adapter{"ros_gz_image_bridge"};
  std::string payload_adapter{"simulation_payload_stub"};
};

struct TargetConstraints
{
  std::vector<std::string> allowed_dictionaries{"DICT_4X4_50", "DICT_4X4_250", "DICT_5X5_50"};
  std::vector<std::string> allowed_namespaces{"aavc2026", "sar_search", "cargo_delivery"};
  uint32_t marker_id_min{0};
  uint32_t marker_id_max{1000};
};

struct ValidationResult
{
  bool is_valid{true};
  std::vector<std::string> violations;

  void add_violation(const std::string & violation)
  {
    is_valid = false;
    violations.push_back(violation);
  }
};

class EngineeringConfig
{
public:
  std::string schema_version{"1.0.0"};
  std::string deployment_id{"fsd_kmitl_simulation_default"};
  std::string profile{"simulation"};
  uint64_t engineering_config_revision{1};

  SimulationPolicy simulation;
  SafetyPolicy safety;
  RoutePolicy routes;
  AdaptersConfig adapters;
  TargetConstraints target_constraints;

  ValidationResult validate() const;
  std::string compute_canonical_hash() const;

  static EngineeringConfig from_yaml(const YAML::Node & node);
  static EngineeringConfig from_yaml_file(const std::string & file_path);
  static EngineeringConfig from_yaml_string(const std::string & yaml_content);
  static EngineeringConfig create_default_simulation_config();
};

}  // namespace full_self_driving::domain
