#include "domain/target_identity.hpp"

namespace full_self_driving::domain
{

bool TargetIdentity::is_valid() const
{
  return !dictionary.empty() && dictionary.size() <= 32 &&
         !target_namespace.empty() && target_namespace.size() <= 64;
}

bool TargetIdentity::matches(const TargetIdentity & other) const
{
  return *this == other;
}

bool TargetIdentity::matches(const full_self_driving::msg::TargetIdentity & msg) const
{
  return marker_id == msg.marker_id &&
         dictionary == msg.dictionary &&
         target_namespace == msg.target_namespace;
}

full_self_driving::msg::TargetIdentity TargetIdentity::to_msg() const
{
  full_self_driving::msg::TargetIdentity msg;
  msg.marker_id = marker_id;
  msg.dictionary = dictionary;
  msg.target_namespace = target_namespace;
  return msg;
}

TargetIdentity TargetIdentity::from_msg(const full_self_driving::msg::TargetIdentity & msg)
{
  return TargetIdentity(msg.marker_id, msg.dictionary, msg.target_namespace);
}

std::ostream & operator<<(std::ostream & os, const TargetIdentity & id)
{
  os << "TargetIdentity(id=" << id.marker_id
     << ", dict='" << id.dictionary
     << "', ns='" << id.target_namespace << "')";
  return os;
}

}  // namespace full_self_driving::domain
