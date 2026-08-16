#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/quaternion.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "full_self_driving/msg/all_id_observation.hpp"
#include "full_self_driving/msg/all_id_observation_batch.hpp"

namespace full_self_driving::perception
{

struct DetectorConfig
{
  std::string dictionary_name{"DICT_4X4_50"};
  double marker_size_m{0.4};
  std::string map_id{"kmitl_airfield"};
  std::string scenario_id{"default_scenario"};
  std::string target_namespace{"aavc2026"};
  std::string camera_frame{"camera_frame"};
  float min_quality{0.0f};
};

struct DetectionResult
{
  full_self_driving::msg::AllIdObservationBatch batch;
  cv::Mat annotated_image;
  bool calibration_valid{false};
  std::string calibration_sha256;
  size_t total_detected{0};
  size_t total_accepted{0};
};

class ArucoDetector
{
public:
  explicit ArucoDetector(const DetectorConfig & config = DetectorConfig());
  ~ArucoDetector() = default;

  void set_config(const DetectorConfig & config);
  const DetectorConfig & get_config() const { return config_; }

  bool update_camera_info(const sensor_msgs::msg::CameraInfo & info);
  bool is_calibrated() const { return calibration_valid_; }
  const std::string & get_calibration_hash() const { return calibration_sha256_; }
  const cv::Mat & get_camera_matrix() const { return camera_matrix_; }
  const cv::Mat & get_dist_coeffs() const { return dist_coeffs_; }

  DetectionResult process_image(
    const cv::Mat & bgr_image,
    const builtin_interfaces::msg::Time & stamp,
    uint64_t monotonic_ns,
    uint64_t sequence = 0,
    uint32_t dropped_before_batch = 0);

  static int dictionary_name_to_id(const std::string & name);
  static std::string dictionary_id_to_name(int id);
  static std::string compute_calibration_hash(const sensor_msgs::msg::CameraInfo & info);
  static std::array<double, 36> compute_covariance(double distance, double marker_size);
  static float compute_quality(const std::vector<cv::Point2f> & corners, double distance, double marker_size);

private:
  void initialize_detector();

  DetectorConfig config_;
  std::unique_ptr<cv::aruco::ArucoDetector> detector_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  bool calibration_valid_{false};
  std::string calibration_sha256_;
};

}  // namespace full_self_driving::perception
