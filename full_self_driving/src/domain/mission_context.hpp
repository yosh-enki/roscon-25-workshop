#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "domain/target_identity.hpp"
#include "domain/engineering_config.hpp"

namespace full_self_driving::domain
{

enum class ConfigState : uint8_t
{
  UNKNOWN = 0,
  STARTUP = 1,
  STANDBY = 2,
  CONFIGURING = 3,
  VALIDATING = 4,
  COMMITTED = 5,
  READY_FOR_OWNMODE = 6,
  LOCKED = 7,
  COMPLETE = 8,
  CONFIG_INVALID = 9,
  RECOVERY_REQUIRED = 10
};

struct OperatorSelection
{
  uint64_t selection_revision{1};
  std::string map_id{"kmitl_airfield"};
  std::string scenario_id{"default_scenario"};
  std::optional<TargetIdentity> target;
  std::string plan_artifact_id;
  std::string working_plan_id;
  std::string resolved_config_hash;
  uint64_t selected_monotonic_ns{0};
};

struct ValidationReport
{
  bool is_valid{false};
  std::string token;
  std::vector<std::string> violations;

  void add_violation(const std::string & violation)
  {
    is_valid = false;
    violations.push_back(violation);
  }
};

class MissionContext
{
public:
  explicit MissionContext(const std::string & context_id = "ctx_default");
  ~MissionContext() = default;

  const std::string & get_context_id() const { return context_id_; }
  ConfigState get_state() const { return state_; }
  void set_state(ConfigState s) { state_ = s; }

  bool is_armed() const { return armed_; }
  void set_armed(bool armed);

  bool is_locked() const { return locked_; }

  struct OriginHomePosition
  {
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_msl_m{0.0};
    bool valid{false};
  };

  bool has_origin_home_position() const { return origin_home_.valid; }
  OriginHomePosition get_origin_home_position() const { return origin_home_; }
  void set_origin_home_position(double lat_deg, double lon_deg, double alt_msl_m)
  {
    origin_home_.latitude_deg = lat_deg;
    origin_home_.longitude_deg = lon_deg;
    origin_home_.altitude_msl_m = alt_msl_m;
    origin_home_.valid = true;
  }

  const OperatorSelection & get_selection() const { return selection_; }
  uint64_t get_selection_revision() const { return selection_.selection_revision; }
  uint64_t get_committed_revision() const { return committed_revision_; }

  const std::string & get_resolved_config_hash() const { return resolved_config_hash_; }
  std::shared_ptr<const EngineeringConfig> get_resolved_config() const { return resolved_config_; }

  const std::string & get_mission_id() const { return mission_id_; }
  const std::string & get_sortie_id() const { return sortie_id_; }

  bool set_engineering_config(
    std::shared_ptr<const EngineeringConfig> config,
    std::string * out_error = nullptr);

  bool edit_selection(
    const OperatorSelection & new_sel,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  bool select_map_scenario(
    const std::string & map_id,
    const std::string & scenario_id,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  bool select_target(
    const TargetIdentity & target,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  bool select_plan_artifact(
    const std::string & artifact_id,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  bool select_working_plan(
    const std::string & working_plan_id,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  ValidationReport validate_selection(uint64_t expected_revision);

  bool commit(
    const std::string & validation_token,
    uint64_t expected_revision,
    std::string * out_error = nullptr);

  bool check_readiness(
    bool px4_transport_ok,
    bool storage_ok,
    bool health_ok,
    std::vector<std::string> * out_missing_gates = nullptr);

  bool lock(
    const std::string & mission_id,
    const std::string & sortie_id,
    std::string * out_error = nullptr);

  bool unlock(
    bool is_disarmed,
    std::string * out_error = nullptr);

private:
  std::string context_id_;
  ConfigState state_{ConfigState::STANDBY};
  OperatorSelection selection_;
  uint64_t committed_revision_{0};
  bool locked_{false};
  bool armed_{false};

  std::string mission_id_;
  std::string sortie_id_;
  std::string resolved_config_hash_;
  std::shared_ptr<const EngineeringConfig> resolved_config_;
  std::string validation_token_;
  OriginHomePosition origin_home_;
};

}  // namespace full_self_driving::domain
