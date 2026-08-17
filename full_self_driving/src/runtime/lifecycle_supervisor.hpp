#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace full_self_driving::runtime
{

enum class LifecycleState : uint8_t
{
  UNCONFIGURED = 0,
  INACTIVE = 1,
  ACTIVE = 2,
  FINALIZED = 3,
  ERROR = 4
};

struct SupervisedNodeInfo
{
  std::string name;
  LifecycleState state{LifecycleState::UNCONFIGURED};
  bool is_required{true};
  std::string last_error;
};

class LifecycleSupervisor
{
public:
  LifecycleSupervisor();
  ~LifecycleSupervisor() = default;

  void set_fault_injection(const std::string & node_name, const std::string & transition);
  void clear_fault_injection();

  bool configure_all(std::string * out_failed_node = nullptr, std::string * out_error = nullptr);
  bool activate_all(std::string * out_failed_node = nullptr, std::string * out_error = nullptr);
  bool deactivate_all(std::string * out_failed_node = nullptr, std::string * out_error = nullptr);
  bool shutdown_all(std::string * out_failed_node = nullptr, std::string * out_error = nullptr);

  bool configure_node(const std::string & node_name, std::string * out_error = nullptr);
  bool activate_node(const std::string & node_name, std::string * out_error = nullptr);
  bool deactivate_node(const std::string & node_name, std::string * out_error = nullptr);
  bool shutdown_node(const std::string & node_name, std::string * out_error = nullptr);

  bool is_all_active() const;
  LifecycleState get_node_state(const std::string & node_name) const;
  std::vector<std::string> get_activation_order() const;
  std::vector<std::string> get_shutdown_order() const;

  bool evaluate_runtime_readiness(
    bool config_ok,
    bool persistence_ok,
    bool recovery_clear,
    bool px4_transport_ok,
    std::vector<std::string> * out_missing_gates = nullptr) const;

  const std::vector<std::string> & get_transition_trace() const { return transition_trace_; }
  void clear_transition_trace();

private:
  mutable std::mutex mutex_;
  std::map<std::string, SupervisedNodeInfo> nodes_;
  std::vector<std::string> activation_order_;
  std::vector<std::string> shutdown_order_;
  std::vector<std::string> transition_trace_;

  std::string fault_node_;
  std::string fault_transition_;
};

}  // namespace full_self_driving::runtime
