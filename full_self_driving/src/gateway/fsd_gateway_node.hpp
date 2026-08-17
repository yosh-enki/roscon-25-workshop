#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gateway/fsd_gateway.hpp"
#include "full_self_driving/msg/component_health.hpp"
#include "full_self_driving/srv/select_map_scenario.hpp"
#include "full_self_driving/srv/select_target_identity.hpp"
#include "full_self_driving/srv/select_plan_artifact.hpp"
#include "full_self_driving/srv/create_or_select_working_plan.hpp"
#include "full_self_driving/srv/reset_working_plan.hpp"
#include "full_self_driving/srv/upload_plan_artifact.hpp"
#include "full_self_driving/srv/clear_pad_registry.hpp"
#include "full_self_driving/srv/validate_mission_context.hpp"
#include "full_self_driving/srv/commit_mission_context.hpp"
#include "full_self_driving/srv/resolve_recovery.hpp"

namespace full_self_driving::gateway
{

class GatewayNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit GatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GatewayNode() override = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  FsdGateway & get_gateway() { return gateway_; }

private:
  void publish_health();

  FsdGateway gateway_;
  std::mutex mutex_;

  rclcpp_lifecycle::LifecyclePublisher<full_self_driving::msg::ComponentHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

}  // namespace full_self_driving::gateway
