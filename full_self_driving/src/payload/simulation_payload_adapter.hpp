#pragma once

#include <mutex>
#include <string>
#include "payload/payload_adapter.hpp"

namespace full_self_driving::payload
{

class SimulationPayloadAdapter : public PayloadAdapter
{
public:
  enum FaultMode
  {
    FAULT_NONE = 0,
    FAULT_TIMEOUT = 1,
    FAULT_CONTRADICTORY_FEEDBACK = 2,
    FAULT_HARDWARE_ERROR = 3,
    FAULT_POWER_LOSS = 4
  };

  explicit SimulationPayloadAdapter(const std::string & adapter_id = "sim_payload_01");
  ~SimulationPayloadAdapter() override = default;

  std::string get_adapter_id() const override { return adapter_id_; }
  bool is_healthy() const override { return healthy_; }

  bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) override;

  full_self_driving::msg::PayloadStatus get_status() const override;

  void set_fault_mode(FaultMode mode);
  void set_healthy(bool healthy);
  void set_simulated_latency_us(uint64_t latency_us);

private:
  mutable std::mutex mutex_;
  std::string adapter_id_;
  bool healthy_{true};
  FaultMode fault_mode_{FAULT_NONE};
  uint64_t simulated_latency_us_{15000}; // 15 ms

  uint8_t commanded_state_{full_self_driving::msg::PayloadStatus::COMMAND_SECURED};
  uint8_t feedback_state_{full_self_driving::msg::PayloadStatus::FEEDBACK_SECURED};
  bool cargo_loaded_{true};
  bool secured_{true};
  uint32_t successful_operation_count_{0};
  std::string last_operation_id_{""};
  uint8_t last_operation_result_{full_self_driving::msg::PayloadStatus::RESULT_NONE};
  bool unknown_result_{false};
};

}  // namespace full_self_driving::payload
