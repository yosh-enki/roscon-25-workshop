#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "domain/plan_parser.hpp"

namespace full_self_driving::domain
{

class PlanPrinter
{
public:
  static std::string print(
    const CanonicalSearchRoute & route,
    const std::optional<std::array<double, 2>> & entry_point = std::nullopt,
    size_t next_waypoint_index = 0);

  static std::string update_search_planner_metadata(
    const std::string & original_json,
    const std::array<double, 2> & entry_point,
    size_t next_waypoint_index);
};

}  // namespace full_self_driving::domain
