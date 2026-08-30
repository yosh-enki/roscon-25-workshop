#pragma once

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "domain/target_identity.hpp"
#include "domain/live_target_lock.hpp"
#include "full_self_driving/msg/all_id_observation.hpp"
#include "full_self_driving/msg/all_id_observation_batch.hpp"
#include "full_self_driving/msg/live_target_lock.hpp"

namespace full_self_driving::perception
{

class TargetCoordinator
{
public:
  TargetCoordinator();
  explicit TargetCoordinator(
    const domain::TargetLockPolicy & policy,
    const std::string & map_id = "kmitl_airfield",
    const std::string & scenario_id = "default_scenario");

  void set_selected_target(const domain::TargetIdentity & target);
  void set_selected_targets(const std::vector<domain::TargetIdentity> & targets);
  void add_selected_target(const domain::TargetIdentity & target);
  void clear_selected_target();
  void clear_selected_targets();
  bool has_selected_target() const;
  std::optional<domain::TargetIdentity> get_selected_target() const;
  std::vector<domain::TargetIdentity> get_selected_targets() const;
  bool is_target_allowed(const domain::TargetIdentity & target) const;

  void set_scope(const std::string & map_id, const std::string & scenario_id);
  std::string get_map_id() const;
  std::string get_scenario_id() const;

  void set_policy(const domain::TargetLockPolicy & policy);
  domain::TargetLockPolicy get_policy() const;

  domain::LiveTargetLock process_observation_batch(
    const full_self_driving::msg::AllIdObservationBatch & batch,
    uint64_t monotonic_ns);

  domain::LiveTargetLock check_freshness(uint64_t monotonic_ns);

  domain::LiveTargetLock get_current_lock() const;

  void reset();

private:
  bool validate_observation(
    const full_self_driving::msg::AllIdObservation & obs) const;

  mutable std::mutex mutex_;
  domain::TargetLockPolicy policy_;
  std::string map_id_;
  std::string scenario_id_;
  std::vector<domain::TargetIdentity> selected_targets_;
  std::optional<domain::TargetIdentity> active_latched_target_;

  domain::LiveTargetLock current_lock_;
  std::optional<geometry_msgs::msg::Pose> last_valid_pose_;
  uint64_t last_observation_monotonic_ns_{0};
  uint64_t lock_sequence_{0};
};

}  // namespace full_self_driving::perception
