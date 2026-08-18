#include "hardware_fmu_adapter.hpp"

namespace full_self_driving::adapters
{

HardwareFmuAdapter::HardwareFmuAdapter(
  std::string device_path,
  int baud_rate,
  std::string flow_control)
: device_path_(std::move(device_path)),
  baud_rate_(baud_rate),
  flow_control_(std::move(flow_control)),
  connected_(false),
  healthy_(false)
{
}

std::string HardwareFmuAdapter::get_adapter_id() const
{
  return adapter_id_;
}

bool HardwareFmuAdapter::is_connected() const
{
  // Hardware bringup is deferred pending physical validation package
  return connected_;
}

bool HardwareFmuAdapter::is_healthy() const
{
  // Hardware bringup is deferred; no fake mocks or simulated fallbacks
  return healthy_;
}

std::string HardwareFmuAdapter::get_device_path() const
{
  return device_path_;
}

int HardwareFmuAdapter::get_baud_rate() const
{
  return baud_rate_;
}

std::string HardwareFmuAdapter::get_flow_control() const
{
  return flow_control_;
}

}  // namespace full_self_driving::adapters
