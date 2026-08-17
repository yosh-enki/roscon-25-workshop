#include "runtime/lifecycle_supervisor.hpp"

#include <algorithm>

namespace full_self_driving::runtime
{

LifecycleSupervisor::LifecycleSupervisor()
{
  activation_order_ = {
    "fsd_pad_registry",
    "fsd_perception",
    "fsd_evidence",
    "fsd_gateway"
  };

  shutdown_order_ = {
    "fsd_gateway",
    "fsd_evidence",
    "fsd_perception",
    "fsd_pad_registry"
  };

  for (const auto & name : activation_order_) {
    SupervisedNodeInfo info;
    info.name = name;
    info.state = LifecycleState::UNCONFIGURED;
    info.is_required = true;
    nodes_[name] = info;
  }
}

void LifecycleSupervisor::set_fault_injection(
  const std::string & node_name,
  const std::string & transition)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fault_node_ = node_name;
  fault_transition_ = transition;
}

void LifecycleSupervisor::clear_fault_injection()
{
  std::lock_guard<std::mutex> lock(mutex_);
  fault_node_.clear();
  fault_transition_.clear();
}

void LifecycleSupervisor::clear_transition_trace()
{
  std::lock_guard<std::mutex> lock(mutex_);
  transition_trace_.clear();
}

std::vector<std::string> LifecycleSupervisor::get_activation_order() const
{
  return activation_order_;
}

std::vector<std::string> LifecycleSupervisor::get_shutdown_order() const
{
  return shutdown_order_;
}

LifecycleState LifecycleSupervisor::get_node_state(const std::string & node_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(node_name);
  if (it != nodes_.end()) {
    return it->second.state;
  }
  return LifecycleState::ERROR;
}

bool LifecycleSupervisor::is_all_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & name : activation_order_) {
    auto it = nodes_.find(name);
    if (it == nodes_.end() || it->second.state != LifecycleState::ACTIVE) {
      return false;
    }
  }
  return true;
}

bool LifecycleSupervisor::configure_node(const std::string & node_name, std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(node_name);
  if (it == nodes_.end()) {
    if (out_error) *out_error = "Unknown supervised node: " + node_name;
    return false;
  }

  if (fault_node_ == node_name && fault_transition_ == "configure") {
    it->second.state = LifecycleState::ERROR;
    it->second.last_error = "Injected configure fault";
    transition_trace_.push_back(node_name + ":configure:FAILED");
    if (out_error) *out_error = "Injected configure fault on " + node_name;
    return false;
  }

  it->second.state = LifecycleState::INACTIVE;
  transition_trace_.push_back(node_name + ":configure:SUCCESS");
  return true;
}

bool LifecycleSupervisor::activate_node(const std::string & node_name, std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(node_name);
  if (it == nodes_.end()) {
    if (out_error) *out_error = "Unknown supervised node: " + node_name;
    return false;
  }

  if (it->second.state != LifecycleState::INACTIVE) {
    if (out_error) *out_error = "Node " + node_name + " must be INACTIVE to activate";
    return false;
  }

  if (fault_node_ == node_name && fault_transition_ == "activate") {
    it->second.state = LifecycleState::ERROR;
    it->second.last_error = "Injected activate fault";
    transition_trace_.push_back(node_name + ":activate:FAILED");
    if (out_error) *out_error = "Injected activate fault on " + node_name;
    return false;
  }

  it->second.state = LifecycleState::ACTIVE;
  transition_trace_.push_back(node_name + ":activate:SUCCESS");
  return true;
}

bool LifecycleSupervisor::deactivate_node(const std::string & node_name, std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(node_name);
  if (it == nodes_.end()) {
    if (out_error) *out_error = "Unknown supervised node: " + node_name;
    return false;
  }

  if (fault_node_ == node_name && fault_transition_ == "deactivate") {
    it->second.state = LifecycleState::ERROR;
    it->second.last_error = "Injected deactivate fault";
    transition_trace_.push_back(node_name + ":deactivate:FAILED");
    if (out_error) *out_error = "Injected deactivate fault on " + node_name;
    return false;
  }

  it->second.state = LifecycleState::INACTIVE;
  transition_trace_.push_back(node_name + ":deactivate:SUCCESS");
  return true;
}

bool LifecycleSupervisor::shutdown_node(const std::string & node_name, std::string * out_error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(node_name);
  if (it == nodes_.end()) {
    if (out_error) *out_error = "Unknown supervised node: " + node_name;
    return false;
  }

  it->second.state = LifecycleState::FINALIZED;
  transition_trace_.push_back(node_name + ":shutdown:SUCCESS");
  return true;
}

bool LifecycleSupervisor::configure_all(std::string * out_failed_node, std::string * out_error)
{
  for (const auto & name : activation_order_) {
    if (!configure_node(name, out_error)) {
      if (out_failed_node) *out_failed_node = name;
      return false;
    }
  }
  return true;
}

bool LifecycleSupervisor::activate_all(std::string * out_failed_node, std::string * out_error)
{
  std::vector<std::string> activated_so_far;

  for (const auto & name : activation_order_) {
    if (!activate_node(name, out_error)) {
      if (out_failed_node) *out_failed_node = name;
      // On failure: deactivate already-activated nodes in reverse order
      std::reverse(activated_so_far.begin(), activated_so_far.end());
      for (const auto & prev_name : activated_so_far) {
        deactivate_node(prev_name, nullptr);
      }
      return false;
    }
    activated_so_far.push_back(name);
  }
  return true;
}

bool LifecycleSupervisor::deactivate_all(std::string * out_failed_node, std::string * out_error)
{
  for (const auto & name : shutdown_order_) {
    auto state = get_node_state(name);
    if (state == LifecycleState::ACTIVE) {
      if (!deactivate_node(name, out_error)) {
        if (out_failed_node) *out_failed_node = name;
        return false;
      }
    }
  }
  return true;
}

bool LifecycleSupervisor::shutdown_all(std::string * out_failed_node, std::string * out_error)
{
  deactivate_all(out_failed_node, out_error);
  for (const auto & name : shutdown_order_) {
    if (!shutdown_node(name, out_error)) {
      if (out_failed_node) *out_failed_node = name;
      return false;
    }
  }
  return true;
}

bool LifecycleSupervisor::evaluate_runtime_readiness(
  bool config_ok,
  bool persistence_ok,
  bool recovery_clear,
  bool px4_transport_ok,
  std::vector<std::string> * out_missing_gates) const
{
  std::vector<std::string> missing;

  if (!is_all_active()) {
    missing.push_back("LIFECYCLE_NODES_NOT_ACTIVE");
  }

  if (!config_ok) {
    missing.push_back("CONFIG_NOT_READY");
  }

  if (!persistence_ok) {
    missing.push_back("PERSISTENCE_NOT_HEALTHY");
  }

  if (!recovery_clear) {
    missing.push_back("RECOVERY_REQUIRED");
  }

  if (!px4_transport_ok) {
    missing.push_back("PX4_TRANSPORT_NOT_READY");
  }

  if (out_missing_gates) {
    *out_missing_gates = missing;
  }

  return missing.empty();
}

}  // namespace full_self_driving::runtime
