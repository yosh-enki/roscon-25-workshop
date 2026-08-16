#pragma once

#include <Eigen/Core>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

class TransitIn final : public px4_ros2::ModeBase
{
public:
  explicit TransitIn(rclcpp::Node & node);

  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float) override;

private:
  struct Waypoint
  {
    double latitude_deg;
    double longitude_deg;
  };

  void loadParameters();
  void failMode(const std::string & reason);
  void homePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg);
  void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

  std::optional<float> updateCourseHeading();
  bool waypointReached(const Eigen::Vector3d & target) const;
  bool dataTimedOut() const;

  static constexpr float kHardMaxHorizontalSpeedMps = 10.0F;
  static constexpr float kHardMaxVerticalSpeedMps = 3.0F;
  static constexpr float kHardMaxHeadingRateDegS = 180.0F;
  static constexpr double kHardMaxAltitudeAboveHomeM = 120.0;
  static constexpr float kPi = 3.14159265358979323846F;

  rclcpp::Node & _node;

  std::shared_ptr<px4_ros2::GotoGlobalSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr _home_position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    _vehicle_land_detected_sub;

  std::vector<Waypoint> _waypoints;
  std::size_t _waypoint_index{0};

  double _transit_altitude_above_home_m{0.0};
  double _target_altitude_msl_m{0.0};
  float _arrival_radius_m{2.0F};
  float _max_horizontal_speed_m_s{3.0F};
  float _max_vertical_speed_m_s{1.0F};
  float _max_heading_rate_rad_s{0.785398163F};
  float _course_heading_min_speed_m_s{0.3F};
  float _altitude_tolerance_m{1.0F};
  float _altitude_settle_speed_m_s{0.5F};
  float _data_timeout_s{2.0F};

  std::chrono::steady_clock::time_point _activation_time{};
  std::chrono::steady_clock::time_point _land_state_received_at{};

  double _home_altitude_msl_m{0.0};
  bool _home_position_valid{false};
  bool _land_state_valid{false};
  bool _landed{true};
  bool _waiting_for_fresh_land_state{true};
  bool _parameters_valid{false};
  bool _target_altitude_set{false};
  bool _setpoint_sent_for_current_waypoint{false};
  bool _mode_finished{false};
  bool _last_heading_valid{false};
  float _last_heading_rad{0.0F};
};
