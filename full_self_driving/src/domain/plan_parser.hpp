#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace full_self_driving::domain
{

struct SearchWaypoint
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{15.0};
  uint32_t source_index{0};

  bool operator==(const SearchWaypoint & other) const
  {
    return latitude_deg == other.latitude_deg &&
           longitude_deg == other.longitude_deg &&
           altitude_m == other.altitude_m &&
           source_index == other.source_index;
  }
};

struct CanonicalSearchRoute
{
  std::vector<SearchWaypoint> waypoints;
  float default_altitude_m{15.0f};
  float cruise_speed_m_s{5.0f};
  std::string canonical_route_sha256;

  bool empty() const { return waypoints.empty(); }
  size_t size() const { return waypoints.size(); }
};

struct PlanParseResult
{
  bool is_valid{false};
  std::string error_code;
  std::string error_message;
  CanonicalSearchRoute route;
  std::string raw_content_sha256;
  uint64_t byte_length{0};
  std::string safe_name;

  // Metadata from QGC / SearchPlanner if present in the plan
  bool has_search_planner_metadata{false};
  std::optional<std::array<double, 2>> entry_point;
  size_t next_waypoint_index{0};
};

class PlanParser
{
public:
  static constexpr size_t kMaxArtifactBytes = 8 * 1024 * 1024;  // 8 MiB
  static constexpr size_t kMaxNestingDepth = 32;
  static constexpr size_t kMaxWaypoints = 2000;

  static bool is_safe_basename(const std::string & name);
  static std::string compute_sha256(const std::vector<uint8_t> & bytes);
  static std::string compute_sha256(const std::string & str);
  static std::string compute_canonical_route_hash(const CanonicalSearchRoute & route);

  static PlanParseResult parse_bytes(
    const std::vector<uint8_t> & bytes,
    const std::string & safe_name = "mission.plan",
    size_t max_bytes = kMaxArtifactBytes);

  static PlanParseResult parse_string(
    const std::string & json_text,
    const std::string & safe_name = "mission.plan",
    size_t max_bytes = kMaxArtifactBytes);

  static PlanParseResult parse_file(
    const std::string & file_path,
    size_t max_bytes = kMaxArtifactBytes);
};

}  // namespace full_self_driving::domain
