#pragma once

#include "payload_adapter.hpp"
#include <string>

namespace full_self_driving::payload
{

class HardwarePayloadAdapter : public PayloadAdapter
{
public:
  HardwarePayloadAdapter(
    std::string device_path = "/dev/gpiochip0",
    int pwm_pin = 18,
    int pwm_frequency_hz = 50,
    int feedback_sense_pin = 24);

  ~HardwarePayloadAdapter() override = default;

  std::string get_adapter_id() const override;
  bool is_healthy() const override;

  bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) override;

  full_self_driving::msg::PayloadStatus get_status() const override;

  std::string get_device_path() const;
  int get_pwm_pin() const;
  int get_pwm_frequency_hz() const;
  int get_feedback_sense_pin() const;

private:
  std::string adapter_id_{"gpio_pwm_payload_actuator"};
  std::string device_path_;
  int pwm_pin_;
  int pwm_frequency_hz_;
  int feedback_sense_pin_;
  bool healthy_{false};
  full_self_driving::msg::PayloadStatus current_status_;
};

}  // namespace full_self_driving::payload
