#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "domain/mission_context.hpp"
#include "flight/internal_strategy.hpp"
#include "payload/payload_controller.hpp"
#include "persistence/persistence_manager.hpp"
#include "full_self_driving/msg/payload_status.hpp"

namespace full_self_driving::flight
{

class PayloadOperationStrategy : public InternalStrategy
{
public:
  enum class SubPhase
  {
    EVALUATE_GATES = 0,
    EXECUTE_RELEASE = 1,
    FINISHED = 2
  };

  explicit PayloadOperationStrategy(
    rclcpp::Node & node,
    std::shared_ptr<payload::PayloadController> controller,
    std::shared_ptr<persistence::PersistenceManager> persistence = nullptr,
    std::shared_ptr<domain::MissionContext> context = nullptr,
    const std::string & operation_id = "");

  ~PayloadOperationStrategy() override = default;

  void on_enter() override;
  void on_update(float dt_s) override;
  void on_exit() override;

  bool is_completed() const override { return completed_; }
  StrategyType get_type() const override { return StrategyType::PAYLOAD_OPERATION; }
  std::string get_name() const override { return "PAYLOAD_OPERATION"; }

  uint8_t get_result() const { return result_; }
  SubPhase get_sub_phase() const { return sub_phase_; }

private:
  rclcpp::Node & node_;
  std::shared_ptr<payload::PayloadController> controller_;
  std::shared_ptr<persistence::PersistenceManager> persistence_;
  std::shared_ptr<domain::MissionContext> context_;
  std::string operation_id_;

  SubPhase sub_phase_{SubPhase::EVALUATE_GATES};
  bool completed_{false};
  uint8_t result_{full_self_driving::msg::PayloadStatus::RESULT_NONE};
  full_self_driving::msg::PayloadStatus status_;
  std::string error_message_;
};

}  // namespace full_self_driving::flight
