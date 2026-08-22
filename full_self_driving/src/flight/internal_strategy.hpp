#pragma once

#include <cstdint>
#include <string>
#include <memory>

namespace full_self_driving::flight
{

enum class StrategyType : uint8_t
{
  WAITING_FOR_MODE = 1,
  TAKEOFF = 2,
  TRANSIT_IN = 3,
  ACQUIRE_TARGET = 4,
  DIRECT = 5,
  SEARCH = 6,
  PRECISION_LAND = 7,
  LANDED_VERIFIED = 8,
  PAYLOAD_OPERATION = 9,
  TAKEOFF_AFTER_DELIVERY = 10,
  TRANSIT_OUT = 11,
  RETURN_STRATEGY = 12,
  RETURN_LANDED = 13,
  HOLD = 14,
  ABORT = 15,
  FAILSAFE = 16,
  FAILED = 17
};

inline std::string strategy_type_to_string(StrategyType type)
{
  switch (type) {
    case StrategyType::WAITING_FOR_MODE: return "WAITING_FOR_MODE";
    case StrategyType::TAKEOFF: return "TAKEOFF";
    case StrategyType::TRANSIT_IN: return "TRANSIT_IN";
    case StrategyType::ACQUIRE_TARGET: return "ACQUIRE_TARGET";
    case StrategyType::DIRECT: return "DIRECT";
    case StrategyType::SEARCH: return "SEARCH";
    case StrategyType::PRECISION_LAND: return "PRECISION_LAND";
    case StrategyType::LANDED_VERIFIED: return "LANDED_VERIFIED";
    case StrategyType::PAYLOAD_OPERATION: return "PAYLOAD_OPERATION";
    case StrategyType::TAKEOFF_AFTER_DELIVERY: return "TAKEOFF_AFTER_DELIVERY";
    case StrategyType::TRANSIT_OUT: return "TRANSIT_OUT";
    case StrategyType::RETURN_STRATEGY: return "RETURN_STRATEGY";
    case StrategyType::RETURN_LANDED: return "RETURN_LANDED";
    case StrategyType::HOLD: return "HOLD";
    case StrategyType::ABORT: return "ABORT";
    case StrategyType::FAILSAFE: return "FAILSAFE";
    case StrategyType::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

class InternalStrategy
{
public:
  virtual ~InternalStrategy() = default;

  virtual void on_enter() = 0;
  virtual void on_update(float dt_s) = 0;
  virtual void on_exit() = 0;

  virtual bool is_completed() const = 0;
  virtual bool is_failed() const { return false; }
  virtual std::string failure_reason() const { return ""; }
  virtual StrategyType get_type() const = 0;
  virtual std::string get_name() const = 0;
};

class WaitingForModeStrategy : public InternalStrategy
{
public:
  WaitingForModeStrategy() = default;
  ~WaitingForModeStrategy() override = default;

  void on_enter() override {}
  void on_update(float dt_s) override { (void)dt_s; }
  void on_exit() override {}

  bool is_completed() const override { return false; }
  StrategyType get_type() const override { return StrategyType::WAITING_FOR_MODE; }
  std::string get_name() const override { return "WAITING_FOR_MODE"; }
};

}  // namespace full_self_driving::flight
