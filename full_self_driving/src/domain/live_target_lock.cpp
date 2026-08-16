#include "domain/live_target_lock.hpp"

namespace full_self_driving::domain
{

bool LiveTargetLock::is_fresh(uint64_t now_monotonic_ns, double max_age_s) const
{
  if (received_monotonic_ns == 0 || now_monotonic_ns < received_monotonic_ns) {
    return false;
  }
  uint64_t age_ns = now_monotonic_ns - received_monotonic_ns;
  double age_s = static_cast<double>(age_ns) / 1e9;
  return age_s <= max_age_s;
}

full_self_driving::msg::LiveTargetLock LiveTargetLock::to_msg() const
{
  full_self_driving::msg::LiveTargetLock msg;
  msg.header.stamp = image_time;
  msg.header.frame_id = pose_frame;
  msg.header.sequence = lock_sequence;
  msg.identity = identity.to_msg();
  msg.map_id = map_id;
  msg.scenario_id = scenario_id;
  msg.pose_frame = pose_frame;
  msg.pose = pose;
  msg.covariance = covariance;
  msg.quality = quality;
  msg.consecutive_observations = consecutive_observations;
  msg.image_time = image_time;
  msg.received_monotonic_ns = received_monotonic_ns;
  msg.lock_state = static_cast<uint8_t>(lock_state);
  msg.lock_sequence = lock_sequence;
  return msg;
}

LiveTargetLock LiveTargetLock::from_msg(const full_self_driving::msg::LiveTargetLock & msg)
{
  LiveTargetLock lock;
  lock.identity = TargetIdentity::from_msg(msg.identity);
  lock.map_id = msg.map_id;
  lock.scenario_id = msg.scenario_id;
  lock.pose_frame = msg.pose_frame;
  lock.pose = msg.pose;
  lock.covariance = msg.covariance;
  lock.quality = msg.quality;
  lock.consecutive_observations = msg.consecutive_observations;
  lock.image_time = msg.image_time;
  lock.received_monotonic_ns = msg.received_monotonic_ns;
  lock.lock_state = static_cast<LockState>(msg.lock_state);
  lock.lock_sequence = msg.lock_sequence;
  return lock;
}

const char * to_string(LockState state)
{
  switch (state) {
    case LockState::NONE:
      return "STATE_NONE";
    case LockState::CANDIDATE:
      return "STATE_CANDIDATE";
    case LockState::QUALIFIED:
      return "STATE_QUALIFIED";
    case LockState::STALE:
      return "STATE_STALE";
    case LockState::LOST:
      return "STATE_LOST";
    default:
      return "UNKNOWN";
  }
}

}  // namespace full_self_driving::domain
