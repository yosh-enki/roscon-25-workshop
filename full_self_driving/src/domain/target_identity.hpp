#pragma once

#include <cstdint>
#include <string>
#include <ostream>
#include <tuple>

#include "full_self_driving/msg/target_identity.hpp"

namespace full_self_driving::domain
{

struct TargetIdentity
{
  uint32_t marker_id{0};
  std::string dictionary{"DICT_4X4_50"};
  std::string target_namespace{"aavc2026"};

  TargetIdentity() = default;
  TargetIdentity(uint32_t id, std::string dict, std::string ns)
  : marker_id(id), dictionary(std::move(dict)), target_namespace(std::move(ns))
  {
  }

  bool is_valid() const;
  bool matches(const TargetIdentity & other) const;
  bool matches(const full_self_driving::msg::TargetIdentity & msg) const;

  full_self_driving::msg::TargetIdentity to_msg() const;
  static TargetIdentity from_msg(const full_self_driving::msg::TargetIdentity & msg);

  bool operator==(const TargetIdentity & other) const
  {
    return marker_id == other.marker_id &&
           dictionary == other.dictionary &&
           target_namespace == other.target_namespace;
  }

  bool operator!=(const TargetIdentity & other) const
  {
    return !(*this == other);
  }

  bool operator<(const TargetIdentity & other) const
  {
    return std::tie(marker_id, dictionary, target_namespace) <
           std::tie(other.marker_id, other.dictionary, other.target_namespace);
  }
};

std::ostream & operator<<(std::ostream & os, const TargetIdentity & id);

}  // namespace full_self_driving::domain
