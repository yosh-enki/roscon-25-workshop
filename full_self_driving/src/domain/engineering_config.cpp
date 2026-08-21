#include "domain/engineering_config.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <openssl/evp.h>
#include <sstream>

namespace full_self_driving::domain
{

ValidationResult EngineeringConfig::validate() const
{
  ValidationResult res;

  if (schema_version != "1.0.0") {
    res.add_violation("schema_version must be '1.0.0', got '" + schema_version + "'");
  }

  if (deployment_id.empty() || deployment_id.size() > 64) {
    res.add_violation("deployment_id must be non-empty and <= 64 characters");
  }

  if (profile != "simulation" && profile != "hardware") {
    res.add_violation("profile must be 'simulation' or 'hardware', got '" + profile + "'");
  }

  // Safety checks
  if (!std::isfinite(safety.max_altitude_m) || safety.max_altitude_m <= 0.0) {
    res.add_violation("safety.max_altitude_m must be finite and > 0.0");
  }

  if (!std::isfinite(safety.min_battery_percentage) ||
      safety.min_battery_percentage < 0.0 || safety.min_battery_percentage > 100.0)
  {
    res.add_violation("safety.min_battery_percentage must be between 0.0 and 100.0");
  }

  if (!std::isfinite(safety.target_loss_timeout_s) || safety.target_loss_timeout_s <= 0.0) {
    res.add_violation("safety.target_loss_timeout_s must be finite and > 0.0");
  }

  // Route checks
  if (!std::isfinite(routes.transit_in_speed_m_s) || routes.transit_in_speed_m_s <= 0.0) {
    res.add_violation("routes.transit_in_speed_m_s must be finite and > 0.0");
  }

  if (!std::isfinite(routes.transit_out_speed_m_s) || routes.transit_out_speed_m_s <= 0.0) {
    res.add_violation("routes.transit_out_speed_m_s must be finite and > 0.0");
  }

  if (!std::isfinite(routes.search_altitude_m) || routes.search_altitude_m <= 0.0) {
    res.add_violation("routes.search_altitude_m must be finite and > 0.0");
  }

  if (!std::isfinite(routes.approach_altitude_m) || routes.approach_altitude_m <= 0.0) {
    res.add_violation("routes.approach_altitude_m must be finite and > 0.0");
  }

  if (!std::isfinite(routes.max_horizontal_velocity_m_s) || routes.max_horizontal_velocity_m_s <= 0.0) {
    res.add_violation("routes.max_horizontal_velocity_m_s must be finite and > 0.0");
  }

  if (!std::isfinite(routes.landing_descent_rate_m_s) || routes.landing_descent_rate_m_s <= 0.0) {
    res.add_violation("routes.landing_descent_rate_m_s must be finite and > 0.0");
  }

  if (!std::isfinite(routes.acceptance_radius_m) || routes.acceptance_radius_m <= 0.0) {
    res.add_violation("routes.acceptance_radius_m must be finite and > 0.0");
  }

  if (!std::isfinite(routes.max_yaw_rate_deg_s) || routes.max_yaw_rate_deg_s <= 0.0) {
    res.add_violation("routes.max_yaw_rate_deg_s must be finite and > 0.0");
  }

  // Relationship check
  if (std::isfinite(routes.search_altitude_m) && std::isfinite(routes.approach_altitude_m) &&
      routes.search_altitude_m < routes.approach_altitude_m)
  {
    res.add_violation("routes.search_altitude_m must be >= routes.approach_altitude_m");
  }

  // Adapters check
  if (adapters.px4_transport.empty()) {
    res.add_violation("adapters.px4_transport must not be empty");
  }
  if (adapters.camera_adapter.empty()) {
    res.add_violation("adapters.camera_adapter must not be empty");
  }
  if (adapters.payload_adapter.empty()) {
    res.add_violation("adapters.payload_adapter must not be empty");
  }

  // Target constraints
  if (target_constraints.marker_id_max < target_constraints.marker_id_min) {
    res.add_violation("target_constraints.marker_id_max must be >= marker_id_min");
  }
  if (target_constraints.allowed_dictionaries.empty()) {
    res.add_violation("target_constraints.allowed_dictionaries must not be empty");
  }
  if (target_constraints.allowed_namespaces.empty()) {
    res.add_violation("target_constraints.allowed_namespaces must not be empty");
  }

  return res;
}

std::string EngineeringConfig::compute_canonical_hash() const
{
  std::ostringstream ss;
  ss << "schema_version=" << schema_version << "\n";
  ss << "deployment_id=" << deployment_id << "\n";
  ss << "profile=" << profile << "\n";
  ss << "revision=" << engineering_config_revision << "\n";
  ss << "sim.world=" << simulation.world << "\n";
  ss << "sim.headless=" << (simulation.headless ? "1" : "0") << "\n";
  ss << "sim.use_sim_time=" << (simulation.use_sim_time ? "1" : "0") << "\n";
  ss << "safety.max_altitude_m=" << std::fixed << std::setprecision(6) << safety.max_altitude_m << "\n";
  ss << "safety.min_battery_percentage=" << std::fixed << std::setprecision(6) << safety.min_battery_percentage << "\n";
  ss << "safety.target_loss_timeout_s=" << std::fixed << std::setprecision(6) << safety.target_loss_timeout_s << "\n";
  ss << "safety.emergency_stop_enabled=" << (safety.emergency_stop_enabled ? "1" : "0") << "\n";
  ss << "routes.transit_in_speed_m_s=" << std::fixed << std::setprecision(6) << routes.transit_in_speed_m_s << "\n";
  ss << "routes.transit_out_speed_m_s=" << std::fixed << std::setprecision(6) << routes.transit_out_speed_m_s << "\n";
  ss << "routes.search_altitude_m=" << std::fixed << std::setprecision(6) << routes.search_altitude_m << "\n";
  ss << "routes.approach_altitude_m=" << std::fixed << std::setprecision(6) << routes.approach_altitude_m << "\n";
  ss << "routes.max_horizontal_velocity_m_s=" << std::fixed << std::setprecision(6) << routes.max_horizontal_velocity_m_s << "\n";
  ss << "routes.landing_descent_rate_m_s=" << std::fixed << std::setprecision(6) << routes.landing_descent_rate_m_s << "\n";
  ss << "adapters.px4_transport=" << adapters.px4_transport << "\n";
  ss << "adapters.camera_adapter=" << adapters.camera_adapter << "\n";
  ss << "adapters.payload_adapter=" << adapters.payload_adapter << "\n";
  ss << "target.marker_id_min=" << target_constraints.marker_id_min << "\n";
  ss << "target.marker_id_max=" << target_constraints.marker_id_max << "\n";

  std::vector<std::string> sorted_dicts = target_constraints.allowed_dictionaries;
  std::sort(sorted_dicts.begin(), sorted_dicts.end());
  for (const auto & d : sorted_dicts) {
    ss << "target.dict=" << d << "\n";
  }

  std::vector<std::string> sorted_ns = target_constraints.allowed_namespaces;
  std::sort(sorted_ns.begin(), sorted_ns.end());
  for (const auto & n : sorted_ns) {
    ss << "target.ns=" << n << "\n";
  }

  std::string canonical_str = ss.str();

  unsigned char hash[EVP_MAX_MD_SIZE];
  size_t hash_len = 0;
  EVP_Q_digest(nullptr, "SHA256", nullptr, canonical_str.data(), canonical_str.size(), hash, &hash_len);

  std::ostringstream hex_stream;
  hex_stream << std::hex << std::setfill('0');
  for (size_t i = 0; i < hash_len; ++i) {
    hex_stream << std::setw(2) << static_cast<int>(hash[i]);
  }
  return hex_stream.str();
}

EngineeringConfig EngineeringConfig::from_yaml(const YAML::Node & node)
{
  EngineeringConfig config;

  if (node["schema_version"]) {
    config.schema_version = node["schema_version"].as<std::string>();
  }
  if (node["deployment_id"]) {
    config.deployment_id = node["deployment_id"].as<std::string>();
  }
  if (node["profile"]) {
    config.profile = node["profile"].as<std::string>();
  }
  if (node["engineering_config_revision"]) {
    config.engineering_config_revision = node["engineering_config_revision"].as<uint64_t>();
  }

  if (node["simulation"]) {
    const auto & sim = node["simulation"];
    if (sim["world"]) config.simulation.world = sim["world"].as<std::string>();
    if (sim["headless"]) config.simulation.headless = sim["headless"].as<bool>();
    if (sim["use_sim_time"]) config.simulation.use_sim_time = sim["use_sim_time"].as<bool>();
  }

  if (node["safety"]) {
    const auto & s = node["safety"];
    if (s["max_altitude_m"]) config.safety.max_altitude_m = s["max_altitude_m"].as<double>();
    if (s["min_battery_percentage"]) config.safety.min_battery_percentage = s["min_battery_percentage"].as<double>();
    if (s["target_loss_timeout_s"]) config.safety.target_loss_timeout_s = s["target_loss_timeout_s"].as<double>();
    if (s["emergency_stop_enabled"]) config.safety.emergency_stop_enabled = s["emergency_stop_enabled"].as<bool>();
    // Resilience: support velocity/descent parameters under safety if placed by user
    if (s["landing_descent_rate_m_s"]) config.routes.landing_descent_rate_m_s = s["landing_descent_rate_m_s"].as<double>();
    if (s["max_horizontal_velocity_m_s"]) config.routes.max_horizontal_velocity_m_s = s["max_horizontal_velocity_m_s"].as<double>();
  }

  if (node["routes"]) {
    const auto & r = node["routes"];
    if (r["transit_in_speed_m_s"]) config.routes.transit_in_speed_m_s = r["transit_in_speed_m_s"].as<double>();
    if (r["transit_out_speed_m_s"]) config.routes.transit_out_speed_m_s = r["transit_out_speed_m_s"].as<double>();
    if (r["search_altitude_m"]) config.routes.search_altitude_m = r["search_altitude_m"].as<double>();
    if (r["approach_altitude_m"]) config.routes.approach_altitude_m = r["approach_altitude_m"].as<double>();
    if (r["max_horizontal_velocity_m_s"]) config.routes.max_horizontal_velocity_m_s = r["max_horizontal_velocity_m_s"].as<double>();
    if (r["landing_descent_rate_m_s"]) config.routes.landing_descent_rate_m_s = r["landing_descent_rate_m_s"].as<double>();
    if (r["acceptance_radius_m"]) {
      config.routes.acceptance_radius_m = r["acceptance_radius_m"].as<double>();
    } else if (r["arrival_radius_m"]) {
      config.routes.acceptance_radius_m = r["arrival_radius_m"].as<double>();
    }
    if (r["max_yaw_rate_deg_s"]) {
      config.routes.max_yaw_rate_deg_s = r["max_yaw_rate_deg_s"].as<double>();
    } else if (r["max_heading_rate_deg_s"]) {
      config.routes.max_yaw_rate_deg_s = r["max_heading_rate_deg_s"].as<double>();
    }
  }

  if (node["payload"]) {
    const auto & p = node["payload"];
    if (p["adapter_type"]) config.adapters.payload_adapter = p["adapter_type"].as<std::string>();
  }

  if (node["adapters"]) {
    const auto & a = node["adapters"];
    if (a["px4_transport"]) config.adapters.px4_transport = a["px4_transport"].as<std::string>();
    if (a["camera_adapter"]) config.adapters.camera_adapter = a["camera_adapter"].as<std::string>();
    if (a["payload_adapter"]) config.adapters.payload_adapter = a["payload_adapter"].as<std::string>();
  }

  if (node["target_constraints"]) {
    const auto & tc = node["target_constraints"];
    if (tc["marker_id_min"]) config.target_constraints.marker_id_min = tc["marker_id_min"].as<uint32_t>();
    if (tc["marker_id_max"]) config.target_constraints.marker_id_max = tc["marker_id_max"].as<uint32_t>();
    if (tc["allowed_dictionaries"] && tc["allowed_dictionaries"].IsSequence()) {
      config.target_constraints.allowed_dictionaries.clear();
      for (const auto & item : tc["allowed_dictionaries"]) {
        config.target_constraints.allowed_dictionaries.push_back(item.as<std::string>());
      }
    }
    if (tc["allowed_namespaces"] && tc["allowed_namespaces"].IsSequence()) {
      config.target_constraints.allowed_namespaces.clear();
      for (const auto & item : tc["allowed_namespaces"]) {
        config.target_constraints.allowed_namespaces.push_back(item.as<std::string>());
      }
    }
  }

  return config;
}

EngineeringConfig EngineeringConfig::from_yaml_file(const std::string & file_path)
{
  YAML::Node node = YAML::LoadFile(file_path);
  return from_yaml(node);
}

EngineeringConfig EngineeringConfig::from_yaml_string(const std::string & yaml_content)
{
  YAML::Node node = YAML::Load(yaml_content);
  return from_yaml(node);
}

EngineeringConfig EngineeringConfig::create_default_simulation_config()
{
  EngineeringConfig config;
  config.schema_version = "1.0.0";
  config.deployment_id = "fsd_kmitl_simulation_default";
  config.profile = "simulation";
  config.engineering_config_revision = 1;

  config.simulation.world = "kmitl_airfield";
  config.simulation.headless = true;
  config.simulation.use_sim_time = true;

  config.safety.max_altitude_m = 30.0;
  config.safety.min_battery_percentage = 20.0;
  config.safety.target_loss_timeout_s = 2.0;
  config.safety.emergency_stop_enabled = true;

  config.routes.transit_in_speed_m_s = 5.0;
  config.routes.transit_out_speed_m_s = 3.0;
  config.routes.search_altitude_m = 15.0;
  config.routes.approach_altitude_m = 5.0;
  config.routes.max_horizontal_velocity_m_s = 5.0;
  config.routes.landing_descent_rate_m_s = 0.5;
  config.routes.acceptance_radius_m = 4.0;
  config.routes.max_yaw_rate_deg_s = 45.0;

  config.adapters.px4_transport = "px4_sitl_uxrce_dds";
  config.adapters.camera_adapter = "ros_gz_image_bridge";
  config.adapters.payload_adapter = "simulation_payload_stub";

  config.target_constraints.allowed_dictionaries = {"DICT_4X4_50", "DICT_4X4_250", "DICT_5X5_50"};
  config.target_constraints.allowed_namespaces = {"aavc2026", "sar_search", "cargo_delivery"};
  config.target_constraints.marker_id_min = 0;
  config.target_constraints.marker_id_max = 1000;

  return config;
}

}  // namespace full_self_driving::domain
