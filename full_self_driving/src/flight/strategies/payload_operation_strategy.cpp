#include "flight/strategies/payload_operation_strategy.hpp"
#include <chrono>

namespace full_self_driving::flight
{

PayloadOperationStrategy::PayloadOperationStrategy(
  rclcpp::Node & node,
  std::shared_ptr<payload::PayloadController> controller,
  std::shared_ptr<persistence::PersistenceManager> persistence,
  std::shared_ptr<domain::MissionContext> context,
  const std::string & operation_id)
: node_(node),
  controller_(std::move(controller)),
  persistence_(std::move(persistence)),
  context_(std::move(context)),
  operation_id_(operation_id)
{
  if (operation_id_.empty()) {
    auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    operation_id_ = "op_release_" + std::to_string(now_ns);
  }
}

void PayloadOperationStrategy::on_enter()
{
  RCLCPP_INFO(node_.get_logger(), "[PAYLOAD_OPERATION] Strategy entered. Op ID: %s", operation_id_.c_str());
  sub_phase_ = SubPhase::EVALUATE_GATES;
  completed_ = false;
  result_ = full_self_driving::msg::PayloadStatus::RESULT_NONE;
}

void PayloadOperationStrategy::on_update(float dt_s)
{
  (void)dt_s;

  if (completed_) {
    return;
  }

  if (sub_phase_ == SubPhase::EVALUATE_GATES) {
    if (!controller_) {
      RCLCPP_ERROR(node_.get_logger(), "[PAYLOAD_OPERATION] PayloadController is missing!");
      result_ = full_self_driving::msg::PayloadStatus::RESULT_FAILURE;
      completed_ = true;
      sub_phase_ = SubPhase::FINISHED;
      return;
    }

    // Persist Durable Intent before commanding actuator
    if (persistence_) {
      persistence::JournalEntry entry;
      entry.event_id = "EVT_PAYLOAD_INTENT";
      entry.idempotency_key = operation_id_;
      entry.mission_id = context_ ? context_->get_mission_id() : "";
      entry.sortie_id = context_ ? context_->get_sortie_id() : "";
      entry.snapshot_hash = context_ ? context_->get_resolved_config_hash() : "";
      entry.component = "payload_controller";
      entry.detail = "Intent to release payload: " + operation_id_;
      entry.timestamp_monotonic_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
      persistence_->append_journal_entry(entry);
    }

    sub_phase_ = SubPhase::EXECUTE_RELEASE;
  }

  if (sub_phase_ == SubPhase::EXECUTE_RELEASE) {
    result_ = controller_->execute_internal_release(operation_id_, status_, &error_message_);

    // Persist Durable Result
    if (persistence_) {
      persistence::JournalEntry entry;
      if (result_ == full_self_driving::msg::PayloadStatus::RESULT_SUCCESS) {
        entry.event_id = "EVT_PAYLOAD_SUCCESS";
      } else if (result_ == full_self_driving::msg::PayloadStatus::RESULT_UNKNOWN) {
        entry.event_id = "EVT_PAYLOAD_UNKNOWN";
      } else {
        entry.event_id = "EVT_PAYLOAD_FAILURE";
      }
      entry.idempotency_key = operation_id_;
      entry.mission_id = context_ ? context_->get_mission_id() : "";
      entry.sortie_id = context_ ? context_->get_sortie_id() : "";
      entry.snapshot_hash = context_ ? context_->get_resolved_config_hash() : "";
      entry.component = "payload_controller";
      entry.detail = "Payload release result=" + std::to_string(result_) + (error_message_.empty() ? "" : ": " + error_message_);
      entry.timestamp_monotonic_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
      persistence_->append_journal_entry(entry);
    }

    RCLCPP_INFO(node_.get_logger(), "[PAYLOAD_OPERATION] Release executed. Result: %u", result_);
    completed_ = true;
    sub_phase_ = SubPhase::FINISHED;
  }
}

void PayloadOperationStrategy::on_exit()
{
  RCLCPP_INFO(node_.get_logger(), "[PAYLOAD_OPERATION] Strategy exited. Result: %u", result_);
}

}  // namespace full_self_driving::flight
