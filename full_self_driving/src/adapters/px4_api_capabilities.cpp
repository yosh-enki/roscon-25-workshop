#include "adapters/px4_api_capabilities.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>

namespace full_self_driving::adapters
{

std::string Px4ApiCapabilityReport::to_string() const
{
  std::ostringstream oss;
  oss << "=== PX4 API Capability Report ===\n";
  oss << "Status: " << (is_valid ? "VALID" : "INVALID") << "\n";
  oss << "Manifest: " << manifest_path << "\n";
  oss << "px4_ros2_cpp Version: " << px4_ros2_cpp_version << "\n";
  oss << "px4_msgs Version: " << px4_msgs_version << "\n";
  oss << "Verified Headers (" << verified_headers.size() << "):\n";
  for (const auto & h : verified_headers) {
    oss << "  - " << h << "\n";
  }
  oss << "Verified Classes (" << verified_classes.size() << "):\n";
  for (const auto & c : verified_classes) {
    oss << "  - " << c << "\n";
  }
  oss << "Verified Messages (" << verified_messages.size() << "):\n";
  for (const auto & m : verified_messages) {
    oss << "  - " << m << "\n";
  }
  if (!errors.empty()) {
    oss << "Errors (" << errors.size() << "):\n";
    for (const auto & e : errors) {
      oss << "  [ERROR] " << e << "\n";
    }
  }
  return oss.str();
}

Px4ApiCapabilityReport Px4ApiCapabilities::verify_api_manifest(const std::string & manifest_path)
{
  Px4ApiCapabilityReport report;
  report.manifest_path = manifest_path;

  if (manifest_path.empty()) {
    report.errors.push_back("Manifest path is empty");
    report.is_valid = false;
    return report;
  }

  std::ifstream fin(manifest_path);
  if (!fin.is_open()) {
    report.errors.push_back("Failed to open manifest file: " + manifest_path);
    report.is_valid = false;
    return report;
  }

  try {
    YAML::Node root = YAML::Load(fin);
    if (!root["schema_version"] || root["schema_version"].as<std::string>() != "1.0") {
      report.errors.push_back("Unsupported or missing schema_version (expected 1.0)");
    }

    if (!root["px4_ros2_cpp"]) {
      report.errors.push_back("Missing px4_ros2_cpp section in manifest");
    } else {
      auto px4_cpp = root["px4_ros2_cpp"];
      report.px4_ros2_cpp_version = px4_cpp["package_version"] ? px4_cpp["package_version"].as<std::string>() : "unknown";

      if (px4_cpp["headers"]) {
        for (const auto & h : px4_cpp["headers"]) {
          report.verified_headers.push_back(h.first.as<std::string>() + ": " + h.second.as<std::string>());
        }
      }

      if (px4_cpp["classes"]) {
        for (const auto & c : px4_cpp["classes"]) {
          report.verified_classes.push_back(c.first.as<std::string>());
        }
      }

      if (!px4_cpp["classes"]["ModeBase"] || !px4_cpp["classes"]["ModeExecutorBase"]) {
        report.errors.push_back("Manifest missing ModeBase or ModeExecutorBase class definitions");
      }
    }

    if (!root["px4_msgs"]) {
      report.errors.push_back("Missing px4_msgs section in manifest");
    } else {
      auto px4_m = root["px4_msgs"];
      report.px4_msgs_version = px4_m["package_version"] ? px4_m["package_version"].as<std::string>() : "unknown";

      if (px4_m["required_messages"]) {
        for (const auto & m : px4_m["required_messages"]) {
          report.verified_messages.push_back(m.as<std::string>());
        }
      }
    }

    report.is_valid = report.errors.empty();
  } catch (const std::exception & e) {
    report.errors.push_back(std::string("YAML parse exception: ") + e.what());
    report.is_valid = false;
  }

  return report;
}

bool Px4ApiCapabilities::verify_message_compatibility(rclcpp::Node & node, const std::string & topic_prefix)
{
  std::vector<px4_ros2::MessageCompatibilityTopic> topics = {
    ALL_PX4_ROS2_MESSAGES
  };
  return px4_ros2::messageCompatibilityCheck(node, topics, topic_prefix);
}

bool Px4ApiCapabilities::run_full_probe(
  rclcpp::Node & node,
  const std::string & manifest_path,
  Px4ApiCapabilityReport * out_report)
{
  auto report = verify_api_manifest(manifest_path);
  if (out_report) {
    *out_report = report;
  }
  if (!report.is_valid) {
    RCLCPP_ERROR(node.get_logger(), "[PROBE] Px4ApiCapabilities validation failed against %s", manifest_path.c_str());
    return false;
  }
  RCLCPP_INFO(node.get_logger(), "[PROBE] Px4ApiCapabilities verified successfully (px4_ros2_cpp=%s, px4_msgs=%s)",
    report.px4_ros2_cpp_version.c_str(), report.px4_msgs_version.c_str());
  return true;
}

}  // namespace full_self_driving::adapters
