#include "domain/route.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>

namespace full_self_driving::domain
{

void Route::add_waypoint(const RoutePoint & point)
{
  waypoints_.push_back(point);
}

void Route::set_waypoints(const std::vector<RoutePoint> & waypoints)
{
  waypoints_ = waypoints;
}

bool Route::validate(std::vector<std::string> * out_errors) const
{
  bool valid = true;

  if (waypoints_.empty()) {
    if (out_errors) {
      out_errors->push_back("Route contains no waypoints");
    }
    valid = false;
  }

  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    if (!waypoints_[i].is_valid()) {
      if (out_errors) {
        out_errors->push_back("Waypoint " + std::to_string(i) + " coordinate or altitude is invalid");
      }
      valid = false;
    }
  }

  if (!std::isfinite(transit_altitude_above_home_m_) || transit_altitude_above_home_m_ < 0.0) {
    if (out_errors) {
      out_errors->push_back("transit_altitude_above_home_m must be finite and >= 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(arrival_radius_m_) || arrival_radius_m_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("arrival_radius_m must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(max_horizontal_speed_m_s_) || max_horizontal_speed_m_s_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("max_horizontal_speed_m_s must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(max_vertical_speed_m_s_) || max_vertical_speed_m_s_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("max_vertical_speed_m_s must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(max_heading_rate_rad_s_) || max_heading_rate_rad_s_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("max_heading_rate must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(altitude_tolerance_m_) || altitude_tolerance_m_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("altitude_tolerance_m must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(altitude_settle_speed_m_s_) || altitude_settle_speed_m_s_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("altitude_settle_speed_m_s must be > 0.0");
    }
    valid = false;
  }

  if (!std::isfinite(data_timeout_s_) || data_timeout_s_ <= 0.0f) {
    if (out_errors) {
      out_errors->push_back("data_timeout_s must be > 0.0");
    }
    valid = false;
  }

  return valid;
}

void Route::set_transit_altitude_above_home_m(double alt)
{
  if (!std::isfinite(alt) || alt < 0.0) {
    transit_altitude_above_home_m_ = 0.0;
  } else if (alt > kHardMaxAltitudeAboveHomeM) {
    transit_altitude_above_home_m_ = kHardMaxAltitudeAboveHomeM;
  } else {
    transit_altitude_above_home_m_ = alt;
  }
}

void Route::set_arrival_radius_m(float radius)
{
  if (std::isfinite(radius) && radius > 0.0f) {
    arrival_radius_m_ = radius;
  }
}

void Route::set_max_horizontal_speed_m_s(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0f) {
    max_horizontal_speed_m_s_ = 1.0f;
  } else if (speed > kHardMaxHorizontalSpeedMps) {
    max_horizontal_speed_m_s_ = kHardMaxHorizontalSpeedMps;
  } else {
    max_horizontal_speed_m_s_ = speed;
  }
}

void Route::set_max_vertical_speed_m_s(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0f) {
    max_vertical_speed_m_s_ = 1.0f;
  } else if (speed > kHardMaxVerticalSpeedMps) {
    max_vertical_speed_m_s_ = kHardMaxVerticalSpeedMps;
  } else {
    max_vertical_speed_m_s_ = speed;
  }
}

void Route::set_max_heading_rate_deg_s(float rate_deg_s)
{
  if (!std::isfinite(rate_deg_s) || rate_deg_s <= 0.0f) {
    max_heading_rate_deg_s_ = 45.0f;
  } else if (rate_deg_s > kHardMaxHeadingRateDegS) {
    max_heading_rate_deg_s_ = kHardMaxHeadingRateDegS;
  } else {
    max_heading_rate_deg_s_ = rate_deg_s;
  }
  max_heading_rate_rad_s_ = max_heading_rate_deg_s_ * kPi / 180.0f;
}

void Route::set_course_heading_min_speed_m_s(float min_speed)
{
  if (std::isfinite(min_speed) && min_speed >= 0.0f) {
    course_heading_min_speed_m_s_ = min_speed;
  }
}

void Route::set_altitude_tolerance_m(float tol)
{
  if (std::isfinite(tol) && tol > 0.0f) {
    altitude_tolerance_m_ = tol;
  }
}

void Route::set_altitude_settle_speed_m_s(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0f) {
    altitude_settle_speed_m_s_ = 0.5f;
  } else if (speed > kHardMaxVerticalSpeedMps) {
    altitude_settle_speed_m_s_ = kHardMaxVerticalSpeedMps;
  } else {
    altitude_settle_speed_m_s_ = speed;
  }
}

void Route::set_data_timeout_s(float timeout)
{
  if (!std::isfinite(timeout) || timeout <= 0.0f) {
    data_timeout_s_ = 2.0f;
  } else if (timeout > kHardMaxDataTimeoutS) {
    data_timeout_s_ = kHardMaxDataTimeoutS;
  } else {
    data_timeout_s_ = timeout;
  }
}

Route Route::create_default_kmitl_transit_in_route()
{
  Route route;
  // Default KMITL TransitIn waypoints matching simulation airfield
  route.add_waypoint(RoutePoint(13.730322, 100.787446, 10.0));
  route.add_waypoint(RoutePoint(13.730397, 100.788694, 10.0));
  route.add_waypoint(RoutePoint(13.730712, 100.788755, 10.0));
  route.set_transit_altitude_above_home_m(10.0);
  route.set_arrival_radius_m(4.0f);
  route.set_max_horizontal_speed_m_s(5.0f);
  route.set_max_vertical_speed_m_s(1.0f);
  route.set_max_heading_rate_deg_s(45.0f);
  route.set_course_heading_min_speed_m_s(0.3f);
  route.set_altitude_tolerance_m(1.0f);
  route.set_altitude_settle_speed_m_s(0.5f);
  route.set_data_timeout_s(2.0f);
  return route;
}

Route Route::from_yaml(const YAML::Node & node)
{
  Route route;

  if (node["transit_in_alt"]) {
    route.set_transit_altitude_above_home_m(node["transit_in_alt"].as<double>());
  }
  if (node["acceptance_radius_m"]) {
    route.set_acceptance_radius_m(node["acceptance_radius_m"].as<float>());
  } else if (node["arrival_radius_m"]) {
    route.set_arrival_radius_m(node["arrival_radius_m"].as<float>());
  }
  if (node["max_horizontal_speed_m_s"]) {
    route.set_max_horizontal_speed_m_s(node["max_horizontal_speed_m_s"].as<float>());
  }
  if (node["max_vertical_speed_m_s"]) {
    route.set_max_vertical_speed_m_s(node["max_vertical_speed_m_s"].as<float>());
  }
  if (node["max_yaw_rate_deg_s"]) {
    route.set_max_yaw_rate_deg_s(node["max_yaw_rate_deg_s"].as<float>());
  } else if (node["max_heading_rate_deg_s"]) {
    route.set_max_heading_rate_deg_s(node["max_heading_rate_deg_s"].as<float>());
  }
  if (node["course_heading_min_speed_m_s"]) {
    route.set_course_heading_min_speed_m_s(node["course_heading_min_speed_m_s"].as<float>());
  }
  if (node["altitude_tolerance_m"]) {
    route.set_altitude_tolerance_m(node["altitude_tolerance_m"].as<float>());
  }
  if (node["altitude_settle_speed_m_s"]) {
    route.set_altitude_settle_speed_m_s(node["altitude_settle_speed_m_s"].as<float>());
  }
  if (node["data_timeout_s"]) {
    route.set_data_timeout_s(node["data_timeout_s"].as<float>());
  }

  // Parse waypoints list
  if (node["waypoints"] && node["waypoints"].IsSequence()) {
    bool has_maps = false;
    for (const auto & val : node["waypoints"]) {
      if (val.IsMap()) {
        has_maps = true;
        double lat = val["latitude_deg"] ? val["latitude_deg"].as<double>() : 0.0;
        double lon = val["longitude_deg"] ? val["longitude_deg"].as<double>() : 0.0;
        double alt = val["altitude_m"] ? val["altitude_m"].as<double>() : route.get_transit_altitude_above_home_m();
        route.add_waypoint(RoutePoint(lat, lon, alt));
      }
    }

    if (!has_maps) {
      std::vector<double> flattened;
      for (const auto & val : node["waypoints"]) {
        if (val.IsScalar()) {
          flattened.push_back(val.as<double>());
        }
      }
      if (flattened.size() % 2 == 0) {
        for (std::size_t i = 0; i < flattened.size(); i += 2) {
          route.add_waypoint(RoutePoint(flattened[i], flattened[i + 1], route.get_transit_altitude_above_home_m()));
        }
      }
    }
  } else if (node["waypoint_latitudes"] && node["waypoint_longitudes"]) {
    auto lats = node["waypoint_latitudes"].as<std::vector<double>>();
    auto lons = node["waypoint_longitudes"].as<std::vector<double>>();
    std::size_t count = std::min(lats.size(), lons.size());
    for (std::size_t i = 0; i < count; ++i) {
      route.add_waypoint(RoutePoint(lats[i], lons[i], route.get_transit_altitude_above_home_m()));
    }
  }

  return route;
}

Route Route::from_yaml_file(const std::string & file_path)
{
  YAML::Node root = YAML::LoadFile(file_path);
  if (root["transit_in"] && root["transit_in"]["ros__parameters"]) {
    return from_yaml(root["transit_in"]["ros__parameters"]);
  }
  return from_yaml(root);
}

Route Route::from_flattened_vector(
  const std::vector<double> & flattened_coords,
  double default_alt)
{
  Route route;
  route.set_transit_altitude_above_home_m(default_alt);
  if (flattened_coords.size() % 2 == 0) {
    for (std::size_t i = 0; i < flattened_coords.size(); i += 2) {
      route.add_waypoint(RoutePoint(flattened_coords[i], flattened_coords[i + 1], default_alt));
    }
  }
  return route;
}

}  // namespace full_self_driving::domain
