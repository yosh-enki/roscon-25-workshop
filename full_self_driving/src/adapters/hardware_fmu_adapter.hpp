#pragma once

#include <string>

namespace full_self_driving::adapters
{

class HardwareFmuAdapter
{
public:
  HardwareFmuAdapter(
    std::string device_path = "/dev/ttyAMA0",
    int baud_rate = 921600,
    std::string flow_control = "none");

  virtual ~HardwareFmuAdapter() = default;

  std::string get_adapter_id() const;
  bool is_connected() const;
  bool is_healthy() const;
  std::string get_device_path() const;
  int get_baud_rate() const;
  std::string get_flow_control() const;

private:
  std::string adapter_id_{"px4_hardware_uart_serial"};
  std::string device_path_;
  int baud_rate_;
  std::string flow_control_;
  bool connected_{false};
  bool healthy_{false};
};

}  // namespace full_self_driving::adapters
