#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "domain/target_identity.hpp"
#include "full_self_driving/msg/all_id_observation.hpp"
#include "full_self_driving/msg/all_id_observation_batch.hpp"
#include "full_self_driving/msg/pad_record.hpp"
#include "full_self_driving/msg/pad_registry_snapshot.hpp"
#include "full_self_driving/msg/pad_registry_status.hpp"
#include "full_self_driving/msg/component_health.hpp"

namespace full_self_driving::registry
{

struct PadKey
{
  std::string map_id;
  std::string scenario_id;
  std::string target_namespace;
  std::string dictionary;
  uint32_t marker_id{0};

  bool operator<(const PadKey & other) const
  {
    return std::tie(map_id, scenario_id, target_namespace, dictionary, marker_id) <
           std::tie(other.map_id, other.scenario_id, other.target_namespace, other.dictionary, other.marker_id);
  }

  bool operator==(const PadKey & other) const
  {
    return map_id == other.map_id &&
           scenario_id == other.scenario_id &&
           target_namespace == other.target_namespace &&
           dictionary == other.dictionary &&
           marker_id == other.marker_id;
  }
};

struct RegistryConfig
{
  float min_quality{0.0f};
  double max_record_age_s{3600.0};
  double max_record_uncertainty_m{50.0};
  std::string default_map_id{"kmitl_airfield"};
  std::string default_scenario_id{"default_scenario"};
  std::string origin_session_id{"session_sim_0"};
};

struct ClearResult
{
  bool success{false};
  std::string message;
  uint64_t committed_revision{0};
};

class PadRegistry
{
public:
  explicit PadRegistry(const RegistryConfig & config = RegistryConfig());
  ~PadRegistry() = default;

  void set_config(const RegistryConfig & config);
  const RegistryConfig & get_config() const { return config_; }

  size_t observe(
    const full_self_driving::msg::AllIdObservationBatch & batch,
    uint64_t monotonic_ns);

  std::optional<full_self_driving::msg::PadRecord> lookup(
    const full_self_driving::msg::TargetIdentity & identity,
    const std::string & map_id,
    const std::string & scenario_id) const;

  std::optional<full_self_driving::msg::PadRecord> lookup(
    const domain::TargetIdentity & identity,
    const std::string & map_id,
    const std::string & scenario_id) const;

  ClearResult clear(
    const std::string & map_id,
    const std::string & scenario_id,
    uint64_t expected_revision,
    const std::string & confirmation,
    bool is_disarmed);

  full_self_driving::msg::PadRegistrySnapshot get_snapshot(
    const std::string & map_id,
    const std::string & scenario_id,
    const builtin_interfaces::msg::Time & stamp,
    uint64_t monotonic_ns) const;

  full_self_driving::msg::PadRegistryStatus get_status(
    const std::string & map_id,
    const std::string & scenario_id,
    const builtin_interfaces::msg::Time & stamp,
    uint64_t monotonic_ns,
    const full_self_driving::msg::ComponentHealth & health,
    bool is_disarmed) const;

  uint64_t get_revision() const;
  size_t size() const;
  size_t size(const std::string & map_id, const std::string & scenario_id) const;

  void insert_record_for_test(const full_self_driving::msg::PadRecord & record);

private:
  bool validate_observation(const full_self_driving::msg::AllIdObservation & obs) const;

  mutable std::mutex mutex_;
  RegistryConfig config_;
  uint64_t revision_{1};
  std::map<PadKey, full_self_driving::msg::PadRecord> records_;
  std::map<PadKey, full_self_driving::msg::PadRecord> backup_records_;

  uint8_t origin_state_{full_self_driving::msg::PadRegistrySnapshot::ORIGIN_LEARNED};
  uint8_t durability_state_{full_self_driving::msg::PadRegistrySnapshot::DURABILITY_SYNCED};
  uint8_t backup_state_{full_self_driving::msg::PadRegistrySnapshot::BACKUP_UNKNOWN};
};

}  // namespace full_self_driving::registry
