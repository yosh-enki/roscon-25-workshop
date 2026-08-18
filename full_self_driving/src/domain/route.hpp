#pragma once

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace full_self_driving::domain
{

struct RoutePoint
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{10.0};  // Relative altitude above home in meters

  RoutePoint() = default;
  RoutePoint(double lat, double lon, double alt = 10.0)
  : latitude_deg(lat), longitude_deg(lon), altitude_m(alt)
  {
  }

  bool is_valid() const
  {
    return std::isfinite(latitude_deg) && std::isfinite(longitude_deg) &&
           std::isfinite(altitude_m) && latitude_deg >= -90.0 &&
           latitude_deg <= 90.0 && longitude_deg >= -180.0 &&
           longitude_deg <= 180.0 && altitude_m >= 0.0 && altitude_m <= 120.0;
  }
};

class Route
{
public:
  // Safety Limits & Hard Caps
  static constexpr float kHardMaxHorizontalSpeedMps = 10.0f;
  static constexpr float kHardMaxVerticalSpeedMps = 3.0f;
  static constexpr float kHardMaxHeadingRateDegS = 180.0f;
  static constexpr double kHardMaxAltitudeAboveHomeM = 120.0;
  static constexpr float kHardMaxDataTimeoutS = 10.0f;
  static constexpr float kPi = 3.14159265358979323846f;

  Route() = default;
  ~Route() = default;

  void add_waypoint(const RoutePoint & point);
  void set_waypoints(const std::vector<RoutePoint> & waypoints);
  const std::vector<RoutePoint> & get_waypoints() const { return waypoints_; }

  std::size_t size() const { return waypoints_.size(); }
  bool empty() const { return waypoints_.empty(); }
  const RoutePoint & operator[](std::size_t index) const { return waypoints_[index]; }
  const RoutePoint & at(std::size_t index) const { return waypoints_.at(index); }

  bool validate(std::vector<std::string> * out_errors = nullptr) const;

  // Clamped Parameter Setters & Getters
  void set_transit_altitude_above_home_m(double alt);
  double get_transit_altitude_above_home_m() const { return transit_altitude_above_home_m_; }

  void set_arrival_radius_m(float radius);
  float get_arrival_radius_m() const { return arrival_radius_m_; }
  void set_acceptance_radius_m(float radius) { set_arrival_radius_m(radius); }
  float get_acceptance_radius_m() const { return arrival_radius_m_; }

  void set_max_horizontal_speed_m_s(float speed);
  float get_max_horizontal_speed_m_s() const { return max_horizontal_speed_m_s_; }

  void set_max_vertical_speed_m_s(float speed);
  float get_max_vertical_speed_m_s() const { return max_vertical_speed_m_s_; }

  void set_max_heading_rate_deg_s(float rate_deg_s);
  float get_max_heading_rate_deg_s() const { return max_heading_rate_deg_s_; }
  float get_max_heading_rate_rad_s() const { return max_heading_rate_rad_s_; }
  void set_max_yaw_rate_deg_s(float rate_deg_s) { set_max_heading_rate_deg_s(rate_deg_s); }
  float get_max_yaw_rate_deg_s() const { return max_heading_rate_deg_s_; }

  void set_course_heading_min_speed_m_s(float min_speed);
  float get_course_heading_min_speed_m_s() const { return course_heading_min_speed_m_s_; }

  void set_altitude_tolerance_m(float tol);
  float get_altitude_tolerance_m() const { return altitude_tolerance_m_; }

  void set_altitude_settle_speed_m_s(float speed);
  float get_altitude_settle_speed_m_s() const { return altitude_settle_speed_m_s_; }

  void set_data_timeout_s(float timeout);
  float get_data_timeout_s() const { return data_timeout_s_; }

  // Factory methods
  static Route create_default_kmitl_transit_in_route();
  static Route create_default_kmitl_transit_out_route();
  static Route from_yaml(const YAML::Node & node);
  static Route from_yaml_file(const std::string & file_path);
  static Route from_flattened_vector(
    const std::vector<double> & flattened_coords,
    double default_alt = 10.0);

private:
  std::vector<RoutePoint> waypoints_;

  double transit_altitude_above_home_m_{10.0};
  float arrival_radius_m_{2.0f};
  float max_horizontal_speed_m_s_{3.0f};
  float max_vertical_speed_m_s_{1.0f};
  float max_heading_rate_deg_s_{45.0f};
  float max_heading_rate_rad_s_{0.785398163f};
  float course_heading_min_speed_m_s_{0.3f};
  float altitude_tolerance_m_{1.0f};
  float altitude_settle_speed_m_s_{0.5f};
  float data_timeout_s_{2.0f};
};

}  // namespace full_self_driving::domain
