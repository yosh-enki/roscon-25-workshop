#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <px4_ros2/common/context.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/vehicle_state/home_position.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>

namespace full_self_driving::adapters
{

struct Px4StateSnapshot
{
  bool local_pos_valid{false};
  bool attitude_valid{false};
  bool global_pos_valid{false};
  bool home_pos_valid{false};
  bool land_detected_valid{false};
  bool vehicle_status_valid{false};

  bool is_armed{false};
  bool is_landed{false};
  uint8_t nav_state{0};

  Eigen::Vector3f local_position_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f local_velocity_ned{0.f, 0.f, 0.f};
  Eigen::Vector3f local_acceleration_ned{0.f, 0.f, 0.f};
  Eigen::Quaternionf attitude{1.f, 0.f, 0.f, 0.f};
  float heading{0.f};
  float distance_ground{0.f};

  Eigen::Vector3d global_position{0.0, 0.0, 0.0};

  Eigen::Vector3f home_local_position{0.f, 0.f, 0.f};
  Eigen::Vector3d home_global_position{0.0, 0.0, 0.0};
  float home_yaw{0.f};

  uint64_t monotonic_timestamp_ns{0};
};

class Px4StateCache
{
public:
  explicit Px4StateCache(px4_ros2::Context & context);
  ~Px4StateCache() = default;

  Px4StateSnapshot capture_snapshot() const;

  bool is_local_position_fresh(std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) const;
  bool is_global_position_fresh(std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) const;
  bool is_home_position_fresh(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000)) const;
  bool is_land_detected_fresh(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;
  bool is_vehicle_status_fresh(std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) const;

  bool is_transport_healthy() const;
  bool is_armed() const;
  bool is_landed() const;
  uint8_t nav_state() const;

  float calculate_heading_from_velocity_or_attitude() const;

  // Direct accessors to underlying px4_ros2 objects
  px4_ros2::OdometryLocalPosition & local_position() { return local_pos_; }
  px4_ros2::OdometryAttitude & attitude() { return attitude_; }
  px4_ros2::OdometryGlobalPosition & global_position() { return global_pos_; }
  px4_ros2::HomePosition & home_position() { return home_pos_; }
  px4_ros2::LandDetected & land_detected() { return land_detected_; }
  px4_ros2::VehicleStatus & vehicle_status() { return vehicle_status_; }

private:
  px4_ros2::Context & context_;
  px4_ros2::OdometryLocalPosition local_pos_;
  px4_ros2::OdometryAttitude attitude_;
  px4_ros2::OdometryGlobalPosition global_pos_;
  px4_ros2::HomePosition home_pos_;
  px4_ros2::LandDetected land_detected_;
  px4_ros2::VehicleStatus vehicle_status_;

  mutable bool home_received_{false};
  mutable Eigen::Vector3f cached_home_local_{0.f, 0.f, 0.f};
  mutable Eigen::Vector3d cached_home_global_{0.0, 0.0, 0.0};
  mutable float cached_home_yaw_{0.f};
};

}  // namespace full_self_driving::adapters
