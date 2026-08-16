#include "fixtures/target_selection_provider.hpp"

#include <chrono>

namespace full_self_driving::test
{

TargetSelectionProvider::TargetSelectionProvider(const rclcpp::NodeOptions & options)
: rclcpp::Node("fsd_target_selection_provider", options)
{
  declare_parameter<int>("marker_id", 0);
  declare_parameter<std::string>("dictionary", "DICT_4X4_50");
  declare_parameter<std::string>("target_namespace", "aavc2026");
  declare_parameter<double>("rate_hz", 1.0);
  declare_parameter<bool>("periodic", true);

  int id = 0;
  get_parameter("marker_id", id);
  marker_id_ = static_cast<uint32_t>(id >= 0 ? id : 0);
  get_parameter("dictionary", dictionary_);
  get_parameter("target_namespace", target_namespace_);
  get_parameter("rate_hz", rate_hz_);
  get_parameter("periodic", periodic_);

  auto reliable_qos = rclcpp::QoS(10).reliable();
  pub_ = create_publisher<full_self_driving::msg::TargetIdentity>(
    "/full_self_driving/target_selection", reliable_qos);

  if (periodic_ && rate_hz_ > 0.0) {
    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TargetSelectionProvider::publish_selection, this));
  } else {
    // Single shot publish after brief startup delay
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        publish_selection();
        timer_->cancel();
      });
  }
}

void TargetSelectionProvider::set_target(
  uint32_t marker_id,
  const std::string & dictionary,
  const std::string & target_namespace)
{
  marker_id_ = marker_id;
  dictionary_ = dictionary;
  target_namespace_ = target_namespace;
  publish_selection();
}

void TargetSelectionProvider::publish_selection()
{
  full_self_driving::msg::TargetIdentity msg;
  msg.marker_id = marker_id_;
  msg.dictionary = dictionary_;
  msg.target_namespace = target_namespace_;

  RCLCPP_INFO(get_logger(), "Publishing target selection: marker_id=%u, dictionary=%s, namespace=%s",
    msg.marker_id, msg.dictionary.c_str(), msg.target_namespace.c_str());

  pub_->publish(msg);
}

}  // namespace full_self_driving::test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<full_self_driving::test::TargetSelectionProvider>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
