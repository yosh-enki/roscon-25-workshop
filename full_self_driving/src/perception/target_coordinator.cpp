#include "perception/target_coordinator.hpp"

#include <cmath>
#include <limits>

namespace full_self_driving::perception
{

TargetCoordinator::TargetCoordinator()
: TargetCoordinator(domain::TargetLockPolicy())
{
}

TargetCoordinator::TargetCoordinator(
  const domain::TargetLockPolicy & policy,
  const std::string & map_id,
  const std::string & scenario_id)
: policy_(policy), map_id_(map_id), scenario_id_(scenario_id)
{
  current_lock_.map_id = map_id_;
  current_lock_.scenario_id = scenario_id_;
  current_lock_.lock_state = domain::LockState::NONE;
  current_lock_.consecutive_observations = 0;
}

void TargetCoordinator::set_selected_target(const domain::TargetIdentity & target)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!selected_target_.has_value() || !selected_target_->matches(target)) {
    selected_target_ = target;
    current_lock_.identity = target;
    current_lock_.lock_state = domain::LockState::NONE;
    current_lock_.consecutive_observations = 0;
    last_valid_pose_.reset();
    last_observation_monotonic_ns_ = 0;
  }
}

void TargetCoordinator::clear_selected_target()
{
  std::lock_guard<std::mutex> lock(mutex_);
  selected_target_.reset();
  current_lock_.identity = domain::TargetIdentity();
  current_lock_.lock_state = domain::LockState::NONE;
  current_lock_.consecutive_observations = 0;
  last_valid_pose_.reset();
  last_observation_monotonic_ns_ = 0;
}

bool TargetCoordinator::has_selected_target() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return selected_target_.has_value();
}

std::optional<domain::TargetIdentity> TargetCoordinator::get_selected_target() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return selected_target_;
}

void TargetCoordinator::set_scope(const std::string & map_id, const std::string & scenario_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (map_id_ != map_id || scenario_id_ != scenario_id) {
    map_id_ = map_id;
    scenario_id_ = scenario_id;
    current_lock_.map_id = map_id_;
    current_lock_.scenario_id = scenario_id_;
    current_lock_.lock_state = domain::LockState::NONE;
    current_lock_.consecutive_observations = 0;
    last_valid_pose_.reset();
    last_observation_monotonic_ns_ = 0;
  }
}

std::string TargetCoordinator::get_map_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return map_id_;
}

std::string TargetCoordinator::get_scenario_id() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return scenario_id_;
}

void TargetCoordinator::set_policy(const domain::TargetLockPolicy & policy)
{
  std::lock_guard<std::mutex> lock(mutex_);
  policy_ = policy;
}

domain::TargetLockPolicy TargetCoordinator::get_policy() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return policy_;
}

bool TargetCoordinator::validate_observation(
  const full_self_driving::msg::AllIdObservation & obs) const
{
  // Scope gate
  if (obs.map_id != map_id_ || obs.scenario_id != scenario_id_) {
    return false;
  }

  // Quality gate
  if (obs.observation_state != full_self_driving::msg::AllIdObservation::QUALITY_ACCEPTED ||
      obs.quality < policy_.minimum_quality)
  {
    return false;
  }

  // Calibration gate
  if (obs.calibration_sha256.empty()) {
    return false;
  }

  // Covariance sanity check
  for (double val : obs.covariance) {
    if (std::isnan(val) || std::isinf(val)) {
      return false;
    }
  }

  // Check diagonals
  if (obs.covariance[0] < 0.0 || obs.covariance[7] < 0.0 || obs.covariance[14] < 0.0 ||
      obs.covariance[21] < 0.0 || obs.covariance[28] < 0.0 || obs.covariance[35] < 0.0)
  {
    return false;
  }

  // Position uncertainty check
  double pos_var = obs.covariance[0] + obs.covariance[7] + obs.covariance[14];
  if (policy_.maximum_position_uncertainty > 0.0 &&
      pos_var > (policy_.maximum_position_uncertainty * policy_.maximum_position_uncertainty))
  {
    return false;
  }

  // Pose position finite check
  if (std::isnan(obs.pose.position.x) || std::isnan(obs.pose.position.y) || std::isnan(obs.pose.position.z) ||
      std::isinf(obs.pose.position.x) || std::isinf(obs.pose.position.y) || std::isinf(obs.pose.position.z))
  {
    return false;
  }

  return true;
}

domain::LiveTargetLock TargetCoordinator::process_observation_batch(
  const full_self_driving::msg::AllIdObservationBatch & batch,
  uint64_t monotonic_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!selected_target_.has_value()) {
    current_lock_.lock_state = domain::LockState::NONE;
    current_lock_.consecutive_observations = 0;
    return current_lock_;
  }

  const full_self_driving::msg::AllIdObservation * matching_obs = nullptr;
  for (const auto & obs : batch.observations) {
    if (obs.identity.marker_id == selected_target_->marker_id &&
        obs.identity.dictionary == selected_target_->dictionary &&
        obs.identity.target_namespace == selected_target_->target_namespace &&
        obs.map_id == map_id_ &&
        obs.scenario_id == scenario_id_)
    {
      matching_obs = &obs;
      break;
    }
  }

  if (!matching_obs || !validate_observation(*matching_obs)) {
    // Target was not seen or failed validation in this batch
    // Check freshness/timeout
    if (last_observation_monotonic_ns_ > 0) {
      uint64_t age_ns = (monotonic_ns >= last_observation_monotonic_ns_) ?
        (monotonic_ns - last_observation_monotonic_ns_) : 0;
      double age_s = static_cast<double>(age_ns) / 1e9;

      if (age_s > policy_.target_loss_timeout_s) {
        if (current_lock_.lock_state != domain::LockState::LOST &&
            current_lock_.lock_state != domain::LockState::NONE)
        {
          current_lock_.lock_state = domain::LockState::LOST;
          current_lock_.consecutive_observations = 0;
          current_lock_.lock_sequence = ++lock_sequence_;
        }
      } else if (age_s > policy_.maximum_pose_age_s) {
        if (current_lock_.lock_state == domain::LockState::QUALIFIED ||
            current_lock_.lock_state == domain::LockState::CANDIDATE)
        {
          current_lock_.lock_state = domain::LockState::STALE;
          current_lock_.lock_sequence = ++lock_sequence_;
        }
      }
    }
    return current_lock_;
  }

  // Matching and valid observation found!
  const auto & obs = *matching_obs;

  // Spatial consistency check
  bool spatially_consistent = true;
  if (current_lock_.consecutive_observations > 0 && last_valid_pose_.has_value()) {
    double dx = obs.pose.position.x - last_valid_pose_->position.x;
    double dy = obs.pose.position.y - last_valid_pose_->position.y;
    double dz = obs.pose.position.z - last_valid_pose_->position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (policy_.spatial_consistency_radius_m > 0.0 && dist > policy_.spatial_consistency_radius_m) {
      spatially_consistent = false;
    }
  }

  if (spatially_consistent) {
    current_lock_.consecutive_observations++;
  } else {
    // Spatial jump outlier: reset to 1 observation
    current_lock_.consecutive_observations = 1;
  }

  if (current_lock_.consecutive_observations >= policy_.minimum_consecutive_observations) {
    current_lock_.lock_state = domain::LockState::QUALIFIED;
  } else {
    current_lock_.lock_state = domain::LockState::CANDIDATE;
  }

  current_lock_.identity = *selected_target_;
  current_lock_.map_id = map_id_;
  current_lock_.scenario_id = scenario_id_;
  current_lock_.pose_frame = obs.pose_frame;
  current_lock_.pose = obs.pose;
  current_lock_.covariance = obs.covariance;
  current_lock_.quality = obs.quality;
  current_lock_.image_time = obs.image_time;
  current_lock_.received_monotonic_ns = monotonic_ns;
  current_lock_.lock_sequence = ++lock_sequence_;

  last_observation_monotonic_ns_ = monotonic_ns;
  last_valid_pose_ = obs.pose;

  return current_lock_;
}

domain::LiveTargetLock TargetCoordinator::check_freshness(uint64_t monotonic_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_observation_monotonic_ns_ == 0 || !selected_target_.has_value()) {
    return current_lock_;
  }

  uint64_t age_ns = (monotonic_ns >= last_observation_monotonic_ns_) ?
    (monotonic_ns - last_observation_monotonic_ns_) : 0;
  double age_s = static_cast<double>(age_ns) / 1e9;

  if (age_s > policy_.target_loss_timeout_s) {
    if (current_lock_.lock_state != domain::LockState::LOST &&
        current_lock_.lock_state != domain::LockState::NONE)
    {
      current_lock_.lock_state = domain::LockState::LOST;
      current_lock_.consecutive_observations = 0;
      current_lock_.lock_sequence = ++lock_sequence_;
    }
  } else if (age_s > policy_.maximum_pose_age_s) {
    if (current_lock_.lock_state == domain::LockState::QUALIFIED ||
        current_lock_.lock_state == domain::LockState::CANDIDATE)
    {
      current_lock_.lock_state = domain::LockState::STALE;
      current_lock_.lock_sequence = ++lock_sequence_;
    }
  }

  return current_lock_;
}

domain::LiveTargetLock TargetCoordinator::get_current_lock() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_lock_;
}

void TargetCoordinator::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_lock_ = domain::LiveTargetLock();
  current_lock_.map_id = map_id_;
  current_lock_.scenario_id = scenario_id_;
  current_lock_.lock_state = domain::LockState::NONE;
  current_lock_.consecutive_observations = 0;
  if (selected_target_.has_value()) {
    current_lock_.identity = *selected_target_;
  }
  last_valid_pose_.reset();
  last_observation_monotonic_ns_ = 0;
}

}  // namespace full_self_driving::perception
