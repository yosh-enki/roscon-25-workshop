#include "domain/plan_printer.hpp"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace full_self_driving::domain
{

std::string PlanPrinter::print(
  const CanonicalSearchRoute & route,
  const std::optional<std::array<double, 2>> & entry_point,
  size_t next_waypoint_index)
{
  std::ostringstream ss;
  ss << std::setprecision(17);

  ss << "{\n";
  ss << "  \"fileType\": \"Plan\",\n";
  ss << "  \"geoFence\": {\n";
  ss << "    \"circles\": [],\n";
  ss << "    \"polygons\": [],\n";
  ss << "    \"version\": 2\n";
  ss << "  },\n";
  ss << "  \"groundStation\": \"QGroundControl\",\n";
  ss << "  \"mission\": {\n";
  ss << "    \"cruiseSpeed\": " << route.cruise_speed_m_s << ",\n";
  ss << "    \"firmwareType\": 12,\n";
  ss << "    \"globalPlanAltitudeMode\": 0,\n";
  ss << "    \"hoverSpeed\": 5,\n";
  ss << "    \"CameraCalc\": {\n";
  ss << "      \"DistanceToSurface\": " << route.default_altitude_m << ",\n";
  ss << "      \"version\": 2\n";
  ss << "    },\n";

  if (entry_point.has_value() || next_waypoint_index > 0) {
    ss << "    \"searchPlanner\": {\n";
    if (entry_point.has_value()) {
      ss << "      \"entryPoint\": [" << (*entry_point)[0] << ", " << (*entry_point)[1] << "],\n";
    }
    ss << "      \"nextWaypointIndex\": " << next_waypoint_index << "\n";
    ss << "    },\n";
  }

  ss << "    \"items\": [\n";
  for (size_t i = 0; i < route.waypoints.size(); ++i) {
    const auto & wp = route.waypoints[i];
    ss << "      {\n";
    ss << "        \"autoContinue\": true,\n";
    ss << "        \"command\": 16,\n";
    ss << "        \"doJumpId\": " << (i + 1) << ",\n";
    ss << "        \"frame\": 3,\n";
    ss << "        \"params\": [\n";
    ss << "          0,\n";
    ss << "          0,\n";
    ss << "          0,\n";
    ss << "          null,\n";
    ss << "          " << wp.latitude_deg << ",\n";
    ss << "          " << wp.longitude_deg << ",\n";
    ss << "          " << wp.altitude_m << "\n";
    ss << "        ],\n";
    ss << "        \"type\": \"SimpleItem\"\n";
    ss << "      }";
    if (i + 1 < route.waypoints.size()) {
      ss << ",";
    }
    ss << "\n";
  }
  ss << "    ],\n";
  ss << "    \"vehicleType\": 2,\n";
  ss << "    \"version\": 2\n";
  ss << "  },\n";
  ss << "  \"rallyPoints\": {\n";
  ss << "    \"points\": [],\n";
  ss << "    \"version\": 2\n";
  ss << "  },\n";
  ss << "  \"version\": 1\n";
  ss << "}\n";

  return ss.str();
}

std::string PlanPrinter::update_search_planner_metadata(
  const std::string & original_json,
  const std::array<double, 2> & entry_point,
  size_t next_waypoint_index)
{
  auto parsed = PlanParser::parse_string(original_json);
  if (!parsed.is_valid) {
    throw std::runtime_error("PlanPrinter: cannot update invalid plan JSON: " + parsed.error_message);
  }

  return print(parsed.route, entry_point, next_waypoint_index);
}

}  // namespace full_self_driving::domain
