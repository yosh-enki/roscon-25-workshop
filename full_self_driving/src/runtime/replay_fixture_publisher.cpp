#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace full_self_driving::runtime
{

class ReplayFixturePublisher : public rclcpp::Node
{
public:
  ReplayFixturePublisher()
  : Node("fsd_replay_fixture_publisher")
  {
    declare_parameter<std::string>("fixture_name", "aruco");
    declare_parameter<std::string>("camera_topic", "/camera");
    declare_parameter<std::string>("camera_info_topic", "/camera_info");
    declare_parameter<double>("rate_hz", 10.0);
    declare_parameter<std::string>("image_path", "");
    declare_parameter<std::string>("camera_info_path", "");

    fixture_name_ = get_parameter("fixture_name").as_string();
    camera_topic_ = get_parameter("camera_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    double rate_hz = get_parameter("rate_hz").as_double();
    std::string img_path = get_parameter("image_path").as_string();
    std::string cam_info_path = get_parameter("camera_info_path").as_string();

    resolve_paths(img_path, cam_info_path);
    load_fixture_data();

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ReplayFixturePublisher::publish_frame, this));

    RCLCPP_INFO(get_logger(), "Replay fixture publisher started for fixture '%s' at %.1f Hz",
      fixture_name_.c_str(), rate_hz);
  }

private:
  void resolve_paths(const std::string & custom_img_path, const std::string & custom_cam_path)
  {
    if (!custom_img_path.empty() && !custom_cam_path.empty()) {
      resolved_image_path_ = custom_img_path;
      resolved_camera_info_path_ = custom_cam_path;
      return;
    }

    std::string pkg_share = ament_index_cpp::get_package_share_directory("full_self_driving");
    std::string rel_fixture = "test/fixtures/prototype_behavior/" + fixture_name_;

    std::vector<std::string> search_roots = {
      pkg_share + "/" + rel_fixture,
      pkg_share + "/share/full_self_driving/" + rel_fixture
    };

    for (const auto & root : search_roots) {
      std::string img = root + "/single_marker_id1.png";
      std::string info = root + "/camera_info.yaml";
      if (std::filesystem::exists(img) && std::filesystem::exists(info)) {
        resolved_image_path_ = img;
        resolved_camera_info_path_ = info;
        RCLCPP_INFO(get_logger(), "Found fixture in: %s", root.c_str());
        return;
      }
    }

    RCLCPP_ERROR(get_logger(), "Could not locate fixture '%s'", fixture_name_.c_str());
  }

  void load_fixture_data()
  {
    if (resolved_image_path_.empty() || !std::filesystem::exists(resolved_image_path_)) {
      RCLCPP_ERROR(get_logger(), "Image file not found: %s", resolved_image_path_.c_str());
      return;
    }
    image_mat_ = cv::imread(resolved_image_path_);
    if (image_mat_.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to read image: %s", resolved_image_path_.c_str());
      return;
    }

    if (resolved_camera_info_path_.empty() || !std::filesystem::exists(resolved_camera_info_path_)) {
      RCLCPP_ERROR(get_logger(), "Camera info file not found: %s", resolved_camera_info_path_.c_str());
      return;
    }

    YAML::Node cam_node = YAML::LoadFile(resolved_camera_info_path_);
    camera_info_msg_.width = cam_node["width"].as<uint32_t>();
    camera_info_msg_.height = cam_node["height"].as<uint32_t>();
    camera_info_msg_.distortion_model = cam_node["distortion_model"].as<std::string>();

    camera_info_msg_.d.clear();
    for (const auto & val : cam_node["d"]) {
      camera_info_msg_.d.push_back(val.as<double>());
    }

    auto k_node = cam_node["k"];
    for (size_t i = 0; i < 9 && i < k_node.size(); ++i) {
      camera_info_msg_.k[i] = k_node[i].as<double>();
    }

    auto r_node = cam_node["r"];
    for (size_t i = 0; i < 9 && i < r_node.size(); ++i) {
      camera_info_msg_.r[i] = r_node[i].as<double>();
    }

    auto p_node = cam_node["p"];
    for (size_t i = 0; i < 12 && i < p_node.size(); ++i) {
      camera_info_msg_.p[i] = p_node[i].as<double>();
    }
  }

  void publish_frame()
  {
    if (image_mat_.empty()) {
      return;
    }

    auto now = get_clock()->now();

    cv_bridge::CvImage cv_img;
    cv_img.header.stamp = now;
    cv_img.header.frame_id = "camera_frame";
    cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img.image = image_mat_;

    camera_info_msg_.header.stamp = now;
    camera_info_msg_.header.frame_id = "camera_frame";

    image_pub_->publish(*cv_img.toImageMsg());
    camera_info_pub_->publish(camera_info_msg_);
  }

  std::string fixture_name_;
  std::string camera_topic_;
  std::string camera_info_topic_;
  std::string resolved_image_path_;
  std::string resolved_camera_info_path_;

  cv::Mat image_mat_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace full_self_driving::runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<full_self_driving::runtime::ReplayFixturePublisher>());
  rclcpp::shutdown();
  return 0;
}
