#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/health_and_arming_checks.hpp>
#include <px4_ros2/components/message_compatibility_check.hpp>
#include <px4_ros2/common/requirement_flags.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/vehicle_state/home_position.hpp>
#include <px4_ros2/vehicle_state/land_detected.hpp>
#include <px4_ros2/vehicle_state/vehicle_status.hpp>

namespace full_self_driving::adapters
{

// Compile-time static assertions verifying exact expected API signatures
static_assert(std::is_base_of_v<px4_ros2::Context, px4_ros2::ModeBase>,
  "ModeBase must inherit from Context");
static_assert(std::is_abstract_v<px4_ros2::ModeBase>,
  "ModeBase must be an abstract class");
static_assert(std::is_abstract_v<px4_ros2::ModeExecutorBase>,
  "ModeExecutorBase must be an abstract class");

struct Px4ApiCapabilityReport
{
  bool is_valid{false};
  std::string manifest_path;
  std::string px4_ros2_cpp_version;
  std::string px4_msgs_version;
  std::vector<std::string> verified_headers;
  std::vector<std::string> verified_classes;
  std::vector<std::string> verified_messages;
  std::vector<std::string> errors;

  std::string to_string() const;
};

class Px4ApiCapabilities
{
public:
  static Px4ApiCapabilityReport verify_api_manifest(const std::string & manifest_path);
  static bool verify_message_compatibility(rclcpp::Node & node, const std::string & topic_prefix = "");
  static bool run_full_probe(rclcpp::Node & node, const std::string & manifest_path, Px4ApiCapabilityReport * out_report = nullptr);
};

}  // namespace full_self_driving::adapters
