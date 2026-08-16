#include "TransitOut.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geodesic.hpp>

#include <cmath>
#include <functional>

namespace
{
constexpr char kModeName[] = "Transit Out";
constexpr char kNodeName[] = "transit_out";
constexpr bool kEnableDebugOutput = true;
}

TransitOut::TransitOut(rclcpp::Node & node)
: ModeBase(node, px4_ros2::ModeBase::Settings{kModeName, false}), _node(node)
{
  _goto_setpoint = std::make_shared<px4_ros2::GotoGlobalSetpointType>(*this);
  _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
  _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

  _home_position_sub = _node.create_subscription<px4_msgs::msg::HomePosition>(
    "/fmu/out/home_position", rclcpp::QoS(1).best_effort(),
    std::bind(&TransitOut::homePositionCallback, this, std::placeholders::_1));

  _vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
    "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
    std::bind(&TransitOut::vehicleLandDetectedCallback, this, std::placeholders::_1));

  loadParameters();
}

void TransitOut::loadParameters()
{
  _node.declare_parameter<double>("transit_out_alt", 10.0);
  _node.declare_parameter<std::vector<double>>("waypoints", std::vector<double>{});
  _node.declare_parameter<std::vector<double>>("waypoint_latitudes", std::vector<double>{});
  _node.declare_parameter<std::vector<double>>("waypoint_longitudes", std::vector<double>{});
  _node.declare_parameter<double>("arrival_radius_m", 2.0);
  _node.declare_parameter<double>("max_horizontal_speed_m_s", 3.0);
  _node.declare_parameter<double>("max_vertical_speed_m_s", 1.0);
  _node.declare_parameter<double>("max_heading_rate_deg_s", 45.0);
  _node.declare_parameter<double>("course_heading_min_speed_m_s", 0.3);
  _node.declare_parameter<double>("altitude_tolerance_m", 1.0);
  _node.declare_parameter<double>("altitude_settle_speed_m_s", 0.5);
  _node.declare_parameter<double>("data_timeout_s", 2.0);

  double max_heading_rate_deg_s = 0.0;
  std::vector<double> flattened_waypoints;
  std::vector<double> waypoint_latitudes;
  std::vector<double> waypoint_longitudes;
  double max_horizontal_speed_m_s = 0.0;

  _node.get_parameter("transit_out_alt", _transit_altitude_above_home_m);
  _node.get_parameter("waypoints", flattened_waypoints);
  _node.get_parameter("waypoint_latitudes", waypoint_latitudes);
  _node.get_parameter("waypoint_longitudes", waypoint_longitudes);
  _node.get_parameter("arrival_radius_m", _arrival_radius_m);
  _node.get_parameter("max_horizontal_speed_m_s", max_horizontal_speed_m_s);
  _node.get_parameter("max_vertical_speed_m_s", _max_vertical_speed_m_s);
  _node.get_parameter("max_heading_rate_deg_s", max_heading_rate_deg_s);
  _node.get_parameter("course_heading_min_speed_m_s", _course_heading_min_speed_m_s);
  _node.get_parameter("altitude_tolerance_m", _altitude_tolerance_m);
  _node.get_parameter("altitude_settle_speed_m_s", _altitude_settle_speed_m_s);
  _node.get_parameter("data_timeout_s", _data_timeout_s);

  bool valid = true;

  if (!std::isfinite(_transit_altitude_above_home_m) ||
      _transit_altitude_above_home_m < 0.0) {
    RCLCPP_ERROR(_node.get_logger(), "transit_out_alt must be finite and non-negative");
    valid = false;
  } else if (_transit_altitude_above_home_m > kHardMaxAltitudeAboveHomeM) {
    RCLCPP_WARN(
      _node.get_logger(),
      "transit_out_alt %.2f exceeds the hard altitude cap %.2f; clamping",
      _transit_altitude_above_home_m, kHardMaxAltitudeAboveHomeM);
    _transit_altitude_above_home_m = kHardMaxAltitudeAboveHomeM;
  }

  if (!std::isfinite(_arrival_radius_m) || _arrival_radius_m <= 0.0F) {
    RCLCPP_ERROR(_node.get_logger(), "arrival_radius_m must be greater than zero");
    valid = false;
  }

  if (!std::isfinite(max_horizontal_speed_m_s) || max_horizontal_speed_m_s <= 0.0) {
    RCLCPP_ERROR(_node.get_logger(), "max_horizontal_speed_m_s must be greater than zero");
    valid = false;
  } else {
    if (max_horizontal_speed_m_s > kHardMaxHorizontalSpeedMps) {
      RCLCPP_WARN(
        _node.get_logger(),
        "max_horizontal_speed_m_s %.2f exceeds the hard safety cap %.2f; clamping",
        max_horizontal_speed_m_s, kHardMaxHorizontalSpeedMps);
      max_horizontal_speed_m_s = kHardMaxHorizontalSpeedMps;
    }
    _max_horizontal_speed_m_s = static_cast<float>(max_horizontal_speed_m_s);
  }

  if (!std::isfinite(_max_vertical_speed_m_s) || _max_vertical_speed_m_s <= 0.0F) {
    RCLCPP_ERROR(_node.get_logger(), "max_vertical_speed_m_s must be greater than zero");
    valid = false;
  } else if (_max_vertical_speed_m_s > kHardMaxVerticalSpeedMps) {
    RCLCPP_WARN(
      _node.get_logger(),
      "max_vertical_speed_m_s %.2f exceeds the hard safety cap %.2f; clamping",
      _max_vertical_speed_m_s, kHardMaxVerticalSpeedMps);
    _max_vertical_speed_m_s = kHardMaxVerticalSpeedMps;
  }

  if (!std::isfinite(max_heading_rate_deg_s) || max_heading_rate_deg_s <= 0.0) {
    RCLCPP_ERROR(_node.get_logger(), "max_heading_rate_deg_s must be greater than zero");
    valid = false;
  } else {
    if (max_heading_rate_deg_s > kHardMaxHeadingRateDegS) {
      RCLCPP_WARN(
        _node.get_logger(),
        "max_heading_rate_deg_s %.2f exceeds the hard safety cap %.2f; clamping",
        max_heading_rate_deg_s, kHardMaxHeadingRateDegS);
      max_heading_rate_deg_s = kHardMaxHeadingRateDegS;
    }
    const float heading_rate_deg_s = static_cast<float>(max_heading_rate_deg_s);
    if (!std::isfinite(heading_rate_deg_s)) {
      RCLCPP_ERROR(_node.get_logger(), "max_heading_rate_deg_s is outside float range");
      valid = false;
    } else {
      _max_heading_rate_rad_s = heading_rate_deg_s * kPi / 180.0F;
    }
  }

  if (!std::isfinite(_course_heading_min_speed_m_s) ||
      _course_heading_min_speed_m_s < 0.0F) {
    RCLCPP_ERROR(
      _node.get_logger(), "course_heading_min_speed_m_s must be finite and non-negative");
    valid = false;
  }

  if (!std::isfinite(_altitude_tolerance_m) || _altitude_tolerance_m <= 0.0F) {
    RCLCPP_ERROR(_node.get_logger(), "altitude_tolerance_m must be greater than zero");
    valid = false;
  }

  if (!std::isfinite(_altitude_settle_speed_m_s) || _altitude_settle_speed_m_s <= 0.0F) {
    RCLCPP_ERROR(_node.get_logger(), "altitude_settle_speed_m_s must be greater than zero");
    valid = false;
  } else if (_altitude_settle_speed_m_s > kHardMaxVerticalSpeedMps) {
    RCLCPP_WARN(
      _node.get_logger(),
      "altitude_settle_speed_m_s %.2f exceeds the hard vertical-speed cap %.2f; clamping",
      _altitude_settle_speed_m_s, kHardMaxVerticalSpeedMps);
    _altitude_settle_speed_m_s = kHardMaxVerticalSpeedMps;
  }

  if (!std::isfinite(_data_timeout_s) || _data_timeout_s <= 0.0F) {
    RCLCPP_ERROR(_node.get_logger(), "data_timeout_s must be greater than zero");
    valid = false;
  } else if (_data_timeout_s > 10.0F) {
    RCLCPP_WARN(_node.get_logger(), "data_timeout_s %.2f exceeds 10 seconds; clamping", _data_timeout_s);
    _data_timeout_s = 10.0F;
  }

  if (!flattened_waypoints.empty()) {
    if (flattened_waypoints.size() % 2U != 0U) {
      RCLCPP_ERROR(
        _node.get_logger(), "waypoints must contain an even number of values: [lat, lon, ...]");
      valid = false;
    } else {
      for (std::size_t index = 0; index < flattened_waypoints.size(); index += 2U) {
        const double latitude = flattened_waypoints[index];
        const double longitude = flattened_waypoints[index + 1U];
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -90.0 ||
            latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
          RCLCPP_ERROR(
            _node.get_logger(), "Invalid waypoint %zu: latitude/longitude is out of range",
            index / 2U);
          valid = false;
          continue;
        }
        _waypoints.push_back({latitude, longitude});
      }
    }
  } else if (!waypoint_latitudes.empty() || !waypoint_longitudes.empty()) {
    if (waypoint_latitudes.size() != waypoint_longitudes.size()) {
      RCLCPP_ERROR(
        _node.get_logger(),
        "waypoint_latitudes and waypoint_longitudes must have the same number of values");
      valid = false;
    } else {
      for (std::size_t index = 0; index < waypoint_latitudes.size(); ++index) {
        const double latitude = waypoint_latitudes[index];
        const double longitude = waypoint_longitudes[index];
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -90.0 ||
            latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
          RCLCPP_ERROR(
            _node.get_logger(), "Invalid waypoint %zu: latitude/longitude is out of range", index);
          valid = false;
          continue;
        }
        _waypoints.push_back({latitude, longitude});
      }
    }
  }

  if (_waypoints.empty()) {
    RCLCPP_ERROR(
      _node.get_logger(),
      "At least one waypoint is required in waypoints or the parallel latitude/longitude arrays");
    valid = false;
  }

  _parameters_valid = valid;

  RCLCPP_INFO(
    _node.get_logger(),
    "Transit Out configured with %zu waypoint(s), altitude %.2f m above home, arrival radius %.2f m, "
    "horizontal speed %.2f m/s",
    _waypoints.size(), _transit_altitude_above_home_m, _arrival_radius_m,
    _max_horizontal_speed_m_s);
}

void TransitOut::onActivate()
{
  _waypoint_index = 0U;
  _target_altitude_msl_m = 0.0;
  _target_altitude_set = false;
  _mode_finished = false;
  _last_heading_valid = false;
  _setpoint_sent_for_current_waypoint = false;
  _activation_time = std::chrono::steady_clock::now();
  _waiting_for_fresh_land_state = true;

  if (_vehicle_local_position->positionXYValid()) {
    const float current_heading = _vehicle_local_position->heading();
    if (std::isfinite(current_heading)) {
      _last_heading_rad = current_heading;
      _last_heading_valid = true;
    }
  }

  RCLCPP_INFO(_node.get_logger(), "Transit Out activated");
}

void TransitOut::onDeactivate()
{
  RCLCPP_INFO(_node.get_logger(), "Transit Out deactivated");
}

void TransitOut::updateSetpoint(float)
{
  if (_mode_finished) {
    return;
  }

  if (!_parameters_valid) {
    failMode("invalid transit parameters");
    return;
  }

  if (!isArmed()) {
    failMode("vehicle is not armed; Transit Out requires an airborne vehicle");
    return;
  }

  if (_waiting_for_fresh_land_state || !_land_state_valid) {
    if (dataTimedOut()) {
      failMode("no fresh vehicle_land_detected sample was received after activation");
    } else {
      RCLCPP_INFO_THROTTLE(
        _node.get_logger(), *_node.get_clock(), 1000,
        "Waiting for a fresh PX4 vehicle_land_detected sample before starting Transit Out");
    }
    return;
  }

  const auto land_state_age = std::chrono::duration_cast<std::chrono::duration<float>>(
    std::chrono::steady_clock::now() - _land_state_received_at);
  if (land_state_age.count() > _data_timeout_s) {
    failMode("vehicle_land_detected data became stale during Transit Out");
    return;
  }

  if (_landed) {
    failMode("vehicle is landed; Transit Out must be selected while airborne");
    return;
  }

  if (!_home_position_valid) {
    if (dataTimedOut()) {
      failMode("no valid PX4 home position was received");
    } else {
      RCLCPP_INFO_THROTTLE(
        _node.get_logger(), *_node.get_clock(), 1000,
        "Waiting for a valid PX4 home position before starting Transit Out");
    }
    return;
  }

  if (!_vehicle_global_position->positionValid() ||
      !_vehicle_local_position->positionXYValid() ||
      !_vehicle_local_position->positionZValid() ||
      !_vehicle_local_position->velocityZValid()) {
    if (_setpoint_sent_for_current_waypoint || dataTimedOut()) {
      failMode("required PX4 position data became invalid during Transit Out");
    } else {
      RCLCPP_INFO_THROTTLE(
        _node.get_logger(), *_node.get_clock(), 1000,
        "Waiting for valid PX4 global and local position data before starting Transit Out");
    }
    return;
  }

  if (!_target_altitude_set) {
    _target_altitude_msl_m = _home_altitude_msl_m + _transit_altitude_above_home_m;
    _target_altitude_set = true;
    RCLCPP_INFO(
      _node.get_logger(), "Transit Out target altitude is %.2f m AMSL", _target_altitude_msl_m);
  }

  if (_waypoint_index >= _waypoints.size()) {
    _mode_finished = true;
    completed(px4_ros2::Result::Success);
    return;
  }

  const Waypoint & waypoint = _waypoints[_waypoint_index];
  const Eigen::Vector3d target{
    waypoint.latitude_deg, waypoint.longitude_deg, _target_altitude_msl_m};

  const std::optional<float> heading = updateCourseHeading();
  _goto_setpoint->update(
    target, heading, _max_horizontal_speed_m_s, _max_vertical_speed_m_s,
    _max_heading_rate_rad_s);

  if (!_setpoint_sent_for_current_waypoint) {
    // Give the GotoGlobalSetpointType map projection one update cycle to publish
    // before allowing a waypoint that is already nearby to complete.
    _setpoint_sent_for_current_waypoint = true;
    return;
  }

  if (!waypointReached(target)) {
    return;
  }

  RCLCPP_INFO(
    _node.get_logger(), "Transit Out reached waypoint %zu/%zu", _waypoint_index + 1U,
    _waypoints.size());
  ++_waypoint_index;
  _setpoint_sent_for_current_waypoint = false;

  if (_waypoint_index >= _waypoints.size()) {
    _mode_finished = true;
    completed(px4_ros2::Result::Success);
  }
}

void TransitOut::homePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg)
{
  _home_position_valid = msg->valid_hpos && msg->valid_alt && std::isfinite(msg->alt);
  if (_home_position_valid) {
    _home_altitude_msl_m = static_cast<double>(msg->alt);
  }
}

void TransitOut::vehicleLandDetectedCallback(
  const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
  _land_state_valid = true;
  _landed = msg->landed;
  _land_state_received_at = std::chrono::steady_clock::now();
  _waiting_for_fresh_land_state = false;
}

std::optional<float> TransitOut::updateCourseHeading()
{
  if (_vehicle_local_position->velocityXYValid()) {
    const Eigen::Vector3f velocity = _vehicle_local_position->velocityNed();
    const float horizontal_speed = std::hypot(velocity.x(), velocity.y());

    if (std::isfinite(horizontal_speed) &&
        horizontal_speed >= _course_heading_min_speed_m_s &&
        std::isfinite(velocity.x()) && std::isfinite(velocity.y())) {
      _last_heading_rad = std::atan2(velocity.y(), velocity.x());
      _last_heading_valid = true;
    }
  }

  if (_last_heading_valid) {
    return _last_heading_rad;
  }

  if (_vehicle_local_position->positionXYValid()) {
    const float current_heading = _vehicle_local_position->heading();
    if (std::isfinite(current_heading)) {
      _last_heading_rad = current_heading;
      _last_heading_valid = true;
      return _last_heading_rad;
    }
  }

  return std::nullopt;
}

bool TransitOut::waypointReached(const Eigen::Vector3d & target) const
{
  const Eigen::Vector3d current_position = _vehicle_global_position->position();
  const float horizontal_distance_m = px4_ros2::horizontalDistanceToGlobalPosition(
    current_position, target);
  const double altitude_error_m = std::abs(current_position.z() - target.z());

  if (!std::isfinite(horizontal_distance_m) ||
      horizontal_distance_m > _arrival_radius_m ||
      !std::isfinite(altitude_error_m) || altitude_error_m > _altitude_tolerance_m) {
    return false;
  }

  if (!_vehicle_local_position->velocityZValid()) {
    return false;
  }

  const float vertical_velocity_m_s = _vehicle_local_position->velocityNed().z();
  return std::isfinite(vertical_velocity_m_s) &&
         std::abs(vertical_velocity_m_s) <= _altitude_settle_speed_m_s;
}

bool TransitOut::dataTimedOut() const
{
  if (_activation_time.time_since_epoch().count() == 0) {
    return false;
  }

  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(
    std::chrono::steady_clock::now() - _activation_time);
  return elapsed.count() > _data_timeout_s;
}

void TransitOut::failMode(const std::string & reason)
{
  if (_mode_finished) {
    return;
  }

  _mode_finished = true;
  RCLCPP_ERROR(_node.get_logger(), "Transit Out failed: %s", reason.c_str());
  completed(px4_ros2::Result::ModeFailureOther);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<TransitOut>>(
    kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
