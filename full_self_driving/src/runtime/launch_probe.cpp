#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class LaunchProbeNode : public rclcpp::Node
{
public:
  LaunchProbeNode()
  : Node("fsd_launch_probe")
  {
    this->declare_parameter<bool>("simulation", true);
    this->declare_parameter<std::string>("world", "kmitl_airfield");
    this->declare_parameter<std::string>("engineering_config", "");
    this->declare_parameter<bool>("probe_simulation_deps", true);

    simulation_ = this->get_parameter("simulation").as_bool();
    world_ = this->get_parameter("world").as_string();
    config_path_ = this->get_parameter("engineering_config").as_string();
    probe_deps_ = this->get_parameter("probe_simulation_deps").as_bool();

    RCLCPP_INFO(this->get_logger(), "==================================================");
    RCLCPP_INFO(this->get_logger(), "Full Self-Driving Launch Probe Initialized");
    RCLCPP_INFO(this->get_logger(), "  Profile: %s", simulation_ ? "simulation" : "hardware");
    RCLCPP_INFO(this->get_logger(), "  World: %s", world_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Engineering Config: %s", config_path_.empty() ? "(default)" : config_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "==================================================");

    if (simulation_ && probe_deps_) {
      // Subscriptions for simulation readiness
      clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(10),
        [this](const rosgraph_msgs::msg::Clock::SharedPtr) {
          clock_received_ = true;
        });

      camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::CameraInfo::SharedPtr) {
          camera_info_received_ = true;
        });

      vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr) {
          vehicle_status_received_ = true;
        });

      timer_ = this->create_wall_timer(2s, std::bind(&LaunchProbeNode::reportStatus, this));
    }
  }

private:
  void reportStatus()
  {
    bool all_ready = clock_received_ && camera_info_received_ && vehicle_status_received_;
    if (all_ready && !reported_ready_) {
      RCLCPP_INFO(this->get_logger(), "[PROBE] *** SIMULATION_PROFILE_READY *** All dependencies live (/clock, /camera_info, PX4 uXRCE-DDS)");
      reported_ready_ = true;
    } else if (!reported_ready_) {
      RCLCPP_INFO(this->get_logger(),
        "[PROBE] Dependency status: /clock=%s, /camera_info=%s, PX4_vehicle_status=%s",
        clock_received_ ? "OK" : "WAITING",
        camera_info_received_ ? "OK" : "WAITING",
        vehicle_status_received_ ? "OK" : "WAITING");
    }
  }

  bool simulation_{true};
  std::string world_;
  std::string config_path_;
  bool probe_deps_{true};

  bool clock_received_{false};
  bool camera_info_received_{false};
  bool vehicle_status_received_{false};
  bool reported_ready_{false};

  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaunchProbeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
