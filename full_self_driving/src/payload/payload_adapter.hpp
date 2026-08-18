#pragma once

#include <string>
#include <memory>
#include "full_self_driving/msg/payload_status.hpp"

namespace full_self_driving::payload
{

class PayloadAdapter
{
public:
  virtual ~PayloadAdapter() = default;

  virtual std::string get_adapter_id() const = 0;
  virtual bool is_healthy() const = 0;

  virtual bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) = 0;

  virtual full_self_driving::msg::PayloadStatus get_status() const = 0;
};

}  // namespace full_self_driving::payload
