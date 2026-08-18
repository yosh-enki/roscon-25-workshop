#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "domain/mission_context.hpp"
#include "payload/payload_adapter.hpp"
#include "full_self_driving/msg/payload_status.hpp"
#include "full_self_driving/srv/prepare_payload.hpp"

namespace full_self_driving::payload
{

class PayloadController
{
public:
  explicit PayloadController(
    std::shared_ptr<PayloadAdapter> adapter,
    std::shared_ptr<domain::MissionContext> context = nullptr);
  ~PayloadController() = default;

  void set_adapter(std::shared_ptr<PayloadAdapter> adapter);
  std::shared_ptr<PayloadAdapter> get_adapter() const;

  void set_mission_context(std::shared_ptr<domain::MissionContext> context);
  std::shared_ptr<domain::MissionContext> get_mission_context() const;

  // Preflight disarmed preparation
  bool prepare(
    uint8_t operation,
    const std::string & request_id,
    uint64_t expected_revision,
    full_self_driving::msg::PayloadStatus & out_status,
    std::string * out_error = nullptr);

  // Internal in-flight release (called only by coordinator / flight strategy)
  uint8_t execute_internal_release(
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status,
    std::string * out_error = nullptr);

  full_self_driving::msg::PayloadStatus get_status() const;
  bool is_ready_for_sortie(std::string * out_reason = nullptr) const;

  void reset_idempotency();

private:
  mutable std::mutex mutex_;
  std::shared_ptr<PayloadAdapter> adapter_;
  std::shared_ptr<domain::MissionContext> context_;

  std::map<std::string, full_self_driving::msg::PayloadStatus> idempotency_records_;
};

}  // namespace full_self_driving::payload
