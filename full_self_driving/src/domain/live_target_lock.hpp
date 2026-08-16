#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "domain/target_identity.hpp"
#include "full_self_driving/msg/live_target_lock.hpp"

namespace full_self_driving::domain
{

enum class LockState : uint8_t
{
  NONE = full_self_driving::msg::LiveTargetLock::STATE_NONE,
  CANDIDATE = full_self_driving::msg::LiveTargetLock::STATE_CANDIDATE,
  QUALIFIED = full_self_driving::msg::LiveTargetLock::STATE_QUALIFIED,
  STALE = full_self_driving::msg::LiveTargetLock::STATE_STALE,
  LOST = full_self_driving::msg::LiveTargetLock::STATE_LOST
};

struct TargetLockPolicy
{
  float minimum_quality{0.1f};
  double maximum_pose_age_s{0.5};
  uint32_t minimum_consecutive_observations{2};
  double maximum_position_uncertainty{10.0};
  double maximum_orientation_uncertainty{10.0};
  std::string required_frame{"camera_frame"};
  double spatial_consistency_radius_m{5.0};
  double target_loss_timeout_s{2.0};
};

struct LiveTargetLock
{
  TargetIdentity identity;
  std::string map_id{"kmitl_airfield"};
  std::string scenario_id{"default_scenario"};
  std::string pose_frame{"camera_frame"};
  geometry_msgs::msg::Pose pose;
  std::array<double, 36> covariance{};
  float quality{0.0f};
  uint32_t consecutive_observations{0};
  builtin_interfaces::msg::Time image_time;
  uint64_t received_monotonic_ns{0};
  LockState lock_state{LockState::NONE};
  uint64_t lock_sequence{0};

  bool is_qualified() const { return lock_state == LockState::QUALIFIED; }
  bool is_candidate() const { return lock_state == LockState::CANDIDATE; }
  bool is_stale() const { return lock_state == LockState::STALE; }
  bool is_lost() const { return lock_state == LockState::LOST; }

  bool is_fresh(uint64_t now_monotonic_ns, double max_age_s) const;

  full_self_driving::msg::LiveTargetLock to_msg() const;
  static LiveTargetLock from_msg(const full_self_driving::msg::LiveTargetLock & msg);
};

const char * to_string(LockState state);

}  // namespace full_self_driving::domain
