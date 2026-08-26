#include "registry/pad_registry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace full_self_driving::registry
{

PadRegistry::PadRegistry(const RegistryConfig & config)
: config_(config)
{
}

void PadRegistry::set_config(const RegistryConfig & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

bool PadRegistry::validate_observation(const full_self_driving::msg::AllIdObservation & obs) const
{
  if (obs.map_id.empty() || obs.scenario_id.empty()) {
    return false;
  }
  if (obs.identity.dictionary.empty() || obs.identity.target_namespace.empty()) {
    return false;
  }
  if (obs.observation_state != full_self_driving::msg::AllIdObservation::QUALITY_ACCEPTED) {
    return false;
  }
  if (obs.quality < config_.min_quality) {
    return false;
  }
  if (obs.calibration_sha256.empty()) {
    return false;
  }
  if (std::isnan(obs.pose.position.x) || std::isnan(obs.pose.position.y) || std::isnan(obs.pose.position.z) ||
      std::isinf(obs.pose.position.x) || std::isinf(obs.pose.position.y) || std::isinf(obs.pose.position.z))
  {
    return false;
  }

  // Covariance finite check
  for (double c : obs.covariance) {
    if (std::isnan(c) || std::isinf(c)) {
      return false;
    }
  }

  // Position variance check
  double pos_var = obs.covariance[0] + obs.covariance[7] + obs.covariance[14];
  if (pos_var < 0.0) {
    return false;
  }
  double uncertainty = std::sqrt(pos_var);
  if (config_.max_record_uncertainty_m > 0.0 && uncertainty > config_.max_record_uncertainty_m) {
    return false;
  }

  return true;
}

size_t PadRegistry::observe(
  const full_self_driving::msg::AllIdObservationBatch & batch,
  uint64_t monotonic_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t accepted_count = 0;

  for (const auto & obs : batch.observations) {
    if (!validate_observation(obs)) {
      continue;
    }

    PadKey key{
      obs.map_id,
      obs.scenario_id,
      obs.identity.target_namespace,
      obs.identity.dictionary,
      obs.identity.marker_id
    };

    double pos_var = obs.covariance[0] + obs.covariance[7] + obs.covariance[14];
    double uncertainty = (pos_var >= 0.0) ? std::sqrt(pos_var) : 0.0;

    auto it = records_.find(key);
    if (it == records_.end()) {
      full_self_driving::msg::PadRecord rec;
      rec.header.stamp = obs.image_time;
      rec.header.frame_id = obs.pose_frame;
      rec.identity = obs.identity;
      rec.map_id = obs.map_id;
      rec.scenario_id = obs.scenario_id;
      rec.latitude_deg = obs.pose.position.x;
      rec.longitude_deg = obs.pose.position.y;
      rec.altitude_m = obs.pose.position.z;
      rec.uncertainty_m = uncertainty;
      rec.quality = obs.quality;
      rec.observation_count = 1;
      rec.first_observed_at = obs.image_time;
      rec.last_observed_at = obs.image_time;
      rec.last_observed_monotonic_ns = monotonic_ns;
      rec.registry_revision = ++revision_;
      rec.calibration_sha256 = obs.calibration_sha256;
      rec.origin_session_id = config_.origin_session_id;

      records_[key] = rec;
    } else {
      auto & rec = it->second;
      rec.observation_count++;
      double alpha = 1.0 / static_cast<double>(rec.observation_count);
      rec.latitude_deg = (1.0 - alpha) * rec.latitude_deg + alpha * obs.pose.position.x;
      rec.longitude_deg = (1.0 - alpha) * rec.longitude_deg + alpha * obs.pose.position.y;
      rec.altitude_m = (1.0 - alpha) * rec.altitude_m + alpha * obs.pose.position.z;
      rec.uncertainty_m = uncertainty;
      rec.quality = std::max(rec.quality, obs.quality);
      rec.last_observed_at = obs.image_time;
      rec.last_observed_monotonic_ns = monotonic_ns;
      rec.registry_revision = ++revision_;
      rec.calibration_sha256 = obs.calibration_sha256;
    }

    durability_state_ = full_self_driving::msg::PadRegistrySnapshot::DURABILITY_DIRTY;
    accepted_count++;
  }

  return accepted_count;
}

std::optional<full_self_driving::msg::PadRecord> PadRegistry::lookup(
  const full_self_driving::msg::TargetIdentity & identity,
  const std::string & map_id,
  const std::string & scenario_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  PadKey key{
    map_id,
    scenario_id,
    identity.target_namespace,
    identity.dictionary,
    identity.marker_id
  };

  auto it = records_.find(key);
  if (it != records_.end()) {
    return it->second;
  }

  // Fallback: match by marker_id across any map scope in active registry
  for (const auto & [_, rec] : records_) {
    if (rec.identity.marker_id == identity.marker_id) {
      if (identity.dictionary.empty() || rec.identity.dictionary.empty() ||
          rec.identity.dictionary == identity.dictionary)
      {
        return rec;
      }
    }
  }

  return std::nullopt;
}

std::optional<full_self_driving::msg::PadRecord> PadRegistry::lookup(
  const domain::TargetIdentity & identity,
  const std::string & map_id,
  const std::string & scenario_id) const
{
  return lookup(identity.to_msg(), map_id, scenario_id);
}

ClearResult PadRegistry::clear(
  const std::string & map_id,
  const std::string & scenario_id,
  uint64_t expected_revision,
  const std::string & confirmation,
  bool is_disarmed)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_disarmed) {
    return {false, "REJECTED_ARMED: Pad registry cannot be cleared while vehicle is armed", revision_};
  }

  if (map_id.empty() || scenario_id.empty()) {
    return {false, "REJECTED_INVALID_SCOPE: map_id and scenario_id must not be empty", revision_};
  }

  if (expected_revision != revision_) {
    return {false, "REJECTED_STALE_REVISION: expected revision " + std::to_string(expected_revision) +
      " does not match current revision " + std::to_string(revision_), revision_};
  }

  if (confirmation != "CONFIRM_CLEAR" && confirmation != "CLEAR") {
    return {false, "REJECTED_INVALID_CONFIRMATION: confirmation token required", revision_};
  }

  // Backup current records
  backup_records_ = records_;
  backup_state_ = full_self_driving::msg::PadRegistrySnapshot::BACKUP_READY;

  // Clear only records matching the requested scope
  for (auto it = records_.begin(); it != records_.end(); ) {
    if (it->first.map_id == map_id && it->first.scenario_id == scenario_id) {
      it = records_.erase(it);
    } else {
      ++it;
    }
  }

  revision_++;
  durability_state_ = full_self_driving::msg::PadRegistrySnapshot::DURABILITY_SYNCED;

  return {true, "SUCCESS", revision_};
}

full_self_driving::msg::PadRegistrySnapshot PadRegistry::get_snapshot(
  const std::string & map_id,
  const std::string & scenario_id,
  const builtin_interfaces::msg::Time & stamp,
  uint64_t monotonic_ns) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  full_self_driving::msg::PadRegistrySnapshot snapshot;
  snapshot.header.stamp = stamp;
  snapshot.header.frame_id = "map";
  snapshot.header.sequence = revision_;
  snapshot.map_id = map_id;
  snapshot.scenario_id = scenario_id;
  snapshot.revision = revision_;
  snapshot.origin_state = origin_state_;
  snapshot.durability_state = durability_state_;
  snapshot.backup_state = backup_state_;
  snapshot.updated_at = stamp;
  snapshot.updated_monotonic_ns = monotonic_ns;

  for (const auto & [key, record] : records_) {
    if (key.map_id == map_id && key.scenario_id == scenario_id) {
      if (snapshot.records.size() < 1024) {
        snapshot.records.push_back(record);
      }
    }
  }

  return snapshot;
}

full_self_driving::msg::PadRegistryStatus PadRegistry::get_status(
  const std::string & map_id,
  const std::string & scenario_id,
  const builtin_interfaces::msg::Time & stamp,
  uint64_t monotonic_ns,
  const full_self_driving::msg::ComponentHealth & health,
  bool is_disarmed) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  full_self_driving::msg::PadRegistryStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = "map";
  status.header.sequence = revision_;
  status.map_id = map_id;
  status.scenario_id = scenario_id;
  status.revision = revision_;
  status.durability_state = durability_state_;
  status.backup_state = backup_state_;
  status.clear_allowed = is_disarmed && (backup_state_ != full_self_driving::msg::PadRegistrySnapshot::BACKUP_FAILED);
  status.component_health = health;

  uint32_t count = 0;
  uint32_t stale_count = 0;
  float min_q = 1.0f;
  uint64_t oldest_age_ms = 0;

  for (const auto & [key, record] : records_) {
    if (key.map_id == map_id && key.scenario_id == scenario_id) {
      count++;
      if (record.quality < min_q) {
        min_q = record.quality;
      }
      uint64_t age_ns = (monotonic_ns >= record.last_observed_monotonic_ns) ?
        (monotonic_ns - record.last_observed_monotonic_ns) : 0;
      double age_s = static_cast<double>(age_ns) / 1e9;
      if (age_s > config_.max_record_age_s) {
        stale_count++;
      }
      uint64_t age_ms = age_ns / 1000000;
      if (age_ms > oldest_age_ms) {
        oldest_age_ms = age_ms;
      }
    }
  }

  status.record_count = count;
  status.stale_record_count = stale_count;
  status.minimum_quality = (count > 0) ? min_q : 0.0f;
  status.oldest_record_age_ms = oldest_age_ms;

  return status;
}

uint64_t PadRegistry::get_revision() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return revision_;
}

size_t PadRegistry::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return records_.size();
}

size_t PadRegistry::size(const std::string & map_id, const std::string & scenario_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  size_t count = 0;
  for (const auto & [key, record] : records_) {
    if (key.map_id == map_id && key.scenario_id == scenario_id) {
      count++;
    }
  }
  return count;
}

void PadRegistry::insert_record_for_test(const full_self_driving::msg::PadRecord & record)
{
  std::lock_guard<std::mutex> lock(mutex_);
  PadKey key{
    record.map_id,
    record.scenario_id,
    record.identity.target_namespace,
    record.identity.dictionary,
    record.identity.marker_id
  };
  records_[key] = record;
  revision_++;
}

}  // namespace full_self_driving::registry
