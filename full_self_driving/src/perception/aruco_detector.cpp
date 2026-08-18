#include "perception/aruco_detector.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <openssl/evp.h>

namespace full_self_driving::perception
{

ArucoDetector::ArucoDetector(const DetectorConfig & config)
: config_(config)
{
  initialize_detector();
}

void ArucoDetector::set_config(const DetectorConfig & config)
{
  config_ = config;
  initialize_detector();
}

void ArucoDetector::initialize_detector()
{
  int dict_id = dictionary_name_to_id(config_.dictionary_name);
  auto dictionary = cv::aruco::getPredefinedDictionary(dict_id);
  auto detector_params = cv::aruco::DetectorParameters();
  detector_ = std::make_unique<cv::aruco::ArucoDetector>(dictionary, detector_params);
}

int ArucoDetector::dictionary_name_to_id(const std::string & name)
{
  if (name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
  if (name == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
  if (name == "DICT_4X4_250") return cv::aruco::DICT_4X4_250;
  if (name == "DICT_4X4_1000") return cv::aruco::DICT_4X4_1000;
  if (name == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
  if (name == "DICT_5X5_100") return cv::aruco::DICT_5X5_100;
  if (name == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
  if (name == "DICT_5X5_1000") return cv::aruco::DICT_5X5_1000;
  if (name == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
  if (name == "DICT_6X6_100") return cv::aruco::DICT_6X6_100;
  if (name == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
  if (name == "DICT_6X6_1000") return cv::aruco::DICT_6X6_1000;
  if (name == "DICT_7X7_50") return cv::aruco::DICT_7X7_50;
  if (name == "DICT_7X7_100") return cv::aruco::DICT_7X7_100;
  if (name == "DICT_7X7_250") return cv::aruco::DICT_7X7_250;
  if (name == "DICT_7X7_1000") return cv::aruco::DICT_7X7_1000;
  if (name == "DICT_ARUCO_ORIGINAL") return cv::aruco::DICT_ARUCO_ORIGINAL;
  if (name == "DICT_APRILTAG_16h5") return cv::aruco::DICT_APRILTAG_16h5;
  if (name == "DICT_APRILTAG_25h9") return cv::aruco::DICT_APRILTAG_25h9;
  if (name == "DICT_APRILTAG_36h10") return cv::aruco::DICT_APRILTAG_36h10;
  if (name == "DICT_APRILTAG_36h11") return cv::aruco::DICT_APRILTAG_36h11;

  // Try parsing as integer if numeric string
  try {
    return std::stoi(name);
  } catch (...) {
    return cv::aruco::DICT_4X4_250;
  }
}

std::string ArucoDetector::dictionary_id_to_name(int id)
{
  switch (id) {
    case cv::aruco::DICT_4X4_50: return "DICT_4X4_50";
    case cv::aruco::DICT_4X4_100: return "DICT_4X4_100";
    case cv::aruco::DICT_4X4_250: return "DICT_4X4_250";
    case cv::aruco::DICT_4X4_1000: return "DICT_4X4_1000";
    case cv::aruco::DICT_5X5_50: return "DICT_5X5_50";
    case cv::aruco::DICT_5X5_100: return "DICT_5X5_100";
    case cv::aruco::DICT_5X5_250: return "DICT_5X5_250";
    case cv::aruco::DICT_5X5_1000: return "DICT_5X5_1000";
    case cv::aruco::DICT_6X6_50: return "DICT_6X6_50";
    case cv::aruco::DICT_6X6_100: return "DICT_6X6_100";
    case cv::aruco::DICT_6X6_250: return "DICT_6X6_250";
    case cv::aruco::DICT_6X6_1000: return "DICT_6X6_1000";
    case cv::aruco::DICT_7X7_50: return "DICT_7X7_50";
    case cv::aruco::DICT_7X7_100: return "DICT_7X7_100";
    case cv::aruco::DICT_7X7_250: return "DICT_7X7_250";
    case cv::aruco::DICT_7X7_1000: return "DICT_7X7_1000";
    case cv::aruco::DICT_ARUCO_ORIGINAL: return "DICT_ARUCO_ORIGINAL";
    case cv::aruco::DICT_APRILTAG_16h5: return "DICT_APRILTAG_16h5";
    case cv::aruco::DICT_APRILTAG_25h9: return "DICT_APRILTAG_25h9";
    case cv::aruco::DICT_APRILTAG_36h10: return "DICT_APRILTAG_36h10";
    case cv::aruco::DICT_APRILTAG_36h11: return "DICT_APRILTAG_36h11";
    default: return "DICT_UNKNOWN";
  }
}

std::string ArucoDetector::compute_calibration_hash(const sensor_msgs::msg::CameraInfo & info)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(8);
  ss << info.width << "x" << info.height << ":" << info.distortion_model << ":";
  for (double d : info.d) {
    ss << d << ",";
  }
  ss << ":";
  for (double k : info.k) {
    ss << k << ",";
  }

  std::string input = ss.str();
  unsigned char hash[EVP_MAX_MD_SIZE];
  unsigned int length = 0;

  EVP_MD_CTX * ctx = EVP_MD_CTX_new();
  if (!ctx) {
    return "";
  }

  if (EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1 ||
      EVP_DigestUpdate(ctx, input.data(), input.size()) != 1 ||
      EVP_DigestFinal_ex(ctx, hash, &length) != 1)
  {
    EVP_MD_CTX_free(ctx);
    return "";
  }
  EVP_MD_CTX_free(ctx);

  std::ostringstream hash_ss;
  for (unsigned int i = 0; i < length; ++i) {
    hash_ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
  }
  return hash_ss.str();
}

bool ArucoDetector::update_camera_info(const sensor_msgs::msg::CameraInfo & info)
{
  if (info.k[0] <= 0.0 || info.width == 0 || info.height == 0) {
    calibration_valid_ = false;
    calibration_sha256_.clear();
    return false;
  }

  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(info.k.data())).clone();
  if (!info.d.empty()) {
    dist_coeffs_ = cv::Mat(static_cast<int>(info.d.size()), 1, CV_64F, const_cast<double *>(info.d.data())).clone();
  } else {
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  }
  calibration_sha256_ = compute_calibration_hash(info);
  calibration_valid_ = true;
  return true;
}

std::array<double, 36> ArucoDetector::compute_covariance(double distance, double marker_size)
{
  std::array<double, 36> cov{};
  cov.fill(0.0);

  // Position variance proportional to distance squared and inversely to marker size
  double scale = (marker_size > 0.0) ? (1.0 / marker_size) : 2.0;
  double var_xy = std::max(0.001, std::pow(0.01 * distance * scale, 2.0));
  double var_z = std::max(0.002, std::pow(0.02 * distance * scale, 2.0));
  double var_rot = std::max(0.005, std::pow(0.01 * distance, 2.0));

  // Row-major 6x6 diagonal: x, y, z, roll, pitch, yaw
  cov[0] = var_xy;    // (0,0) var x
  cov[7] = var_xy;    // (1,1) var y
  cov[14] = var_z;    // (2,2) var z
  cov[21] = var_rot;  // (3,3) var roll
  cov[28] = var_rot;  // (4,4) var pitch
  cov[35] = var_rot;  // (5,5) var yaw

  return cov;
}

float ArucoDetector::compute_quality(
  const std::vector<cv::Point2f> & corners,
  double distance,
  double marker_size)
{
  if (corners.size() != 4 || distance <= 0.0 || marker_size <= 0.0) {
    return 0.0f;
  }

  // Calculate perimeter and area in image pixels
  double p01 = cv::norm(corners[0] - corners[1]);
  double p12 = cv::norm(corners[1] - corners[2]);
  double p23 = cv::norm(corners[2] - corners[3]);
  double p30 = cv::norm(corners[3] - corners[0]);
  double perimeter = p01 + p12 + p23 + p30;

  if (perimeter < 8.0) {
    return 0.1f;
  }

  // Quality metric bounded in [0.0, 1.0]
  float quality = static_cast<float>(std::min(1.0, 1.0 / (1.0 + std::exp(-0.05 * (perimeter - 20.0)))));
  return std::max(0.1f, std::min(1.0f, quality));
}

DetectionResult ArucoDetector::process_image(
  const cv::Mat & bgr_image,
  const builtin_interfaces::msg::Time & stamp,
  uint64_t monotonic_ns,
  uint64_t sequence,
  uint32_t dropped_before_batch)
{
  DetectionResult result;
  result.calibration_valid = calibration_valid_;
  result.calibration_sha256 = calibration_sha256_;
  result.annotated_image = bgr_image.clone();

  result.batch.header.stamp = stamp;
  result.batch.header.frame_id = config_.camera_frame;
  result.batch.header.sequence = sequence;
  result.batch.map_id = config_.map_id;
  result.batch.scenario_id = config_.scenario_id;
  result.batch.dropped_before_batch = dropped_before_batch;

  if (bgr_image.empty()) {
    return result;
  }

  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<std::vector<cv::Point2f>> rejected;

  detector_->detectMarkers(bgr_image, corners, ids, rejected);
  result.total_detected = ids.size();

  if (!ids.empty()) {
    cv::aruco::drawDetectedMarkers(result.annotated_image, corners, ids);
  }

  if (!calibration_valid_ || camera_matrix_.empty() || dist_coeffs_.empty()) {
    return result;
  }

  std::vector<std::vector<cv::Point2f>> undistorted_corners;
  undistorted_corners.reserve(corners.size());

  for (const auto & corner : corners) {
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(
      corner, undistorted, camera_matrix_, dist_coeffs_,
      cv::noArray(), camera_matrix_);
    undistorted_corners.push_back(std::move(undistorted));
  }

  const float half_size = static_cast<float>(config_.marker_size_m / 2.0);
  const std::vector<cv::Point3f> object_points = {
    cv::Point3f(-half_size,  half_size, 0.0f),  // top left
    cv::Point3f( half_size,  half_size, 0.0f),  // top right
    cv::Point3f( half_size, -half_size, 0.0f),  // bottom right
    cv::Point3f(-half_size, -half_size, 0.0f)   // bottom left
  };

  for (size_t i = 0; i < ids.size(); ++i) {
    cv::Vec3d rvec, tvec;
    if (!cv::solvePnP(
        object_points, undistorted_corners[i], camera_matrix_,
        cv::noArray(), rvec, tvec))
    {
      continue;
    }

    if (!std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) ||
        !std::isfinite(rvec[0]) || !std::isfinite(rvec[1]) || !std::isfinite(rvec[2]))
    {
      continue;
    }

    cv::Mat rot_mat;
    cv::Rodrigues(rvec, rot_mat);
    const cv::Quatd quat = cv::Quatd::createFromRotMat(rot_mat).normalize();

    if (!std::isfinite(quat.x) || !std::isfinite(quat.y) ||
        !std::isfinite(quat.z) || !std::isfinite(quat.w))
    {
      continue;
    }

    double distance = cv::norm(tvec);
    float quality = compute_quality(corners[i], distance, config_.marker_size_m);
    if (quality < config_.min_quality) {
      continue;
    }

    cv::drawFrameAxes(
      result.annotated_image, camera_matrix_, cv::noArray(), rvec, tvec,
      static_cast<float>(config_.marker_size_m));

    full_self_driving::msg::AllIdObservation obs;
    obs.header.stamp = stamp;
    obs.header.frame_id = config_.camera_frame;
    obs.header.sequence = sequence;
    obs.identity.marker_id = static_cast<uint32_t>(ids[i]);
    obs.identity.dictionary = config_.dictionary_name;
    obs.identity.target_namespace = config_.target_namespace;
    obs.map_id = config_.map_id;
    obs.scenario_id = config_.scenario_id;
    obs.pose_frame = config_.camera_frame;
    obs.pose.position.x = tvec[0];
    obs.pose.position.y = tvec[1];
    obs.pose.position.z = tvec[2];
    obs.pose.orientation.x = quat.x;
    obs.pose.orientation.y = quat.y;
    obs.pose.orientation.z = quat.z;
    obs.pose.orientation.w = quat.w;
    obs.covariance = compute_covariance(distance, config_.marker_size_m);
    obs.quality = quality;
    obs.image_time = stamp;
    obs.received_monotonic_ns = monotonic_ns;
    obs.calibration_sha256 = calibration_sha256_;
    obs.observation_state = full_self_driving::msg::AllIdObservation::QUALITY_ACCEPTED;

    // Overlay text matching prototype format: "X: <x> Y: <y> Z: <z>"
    if (i == 0) {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "X: " << tvec[0] << " Y: " << tvec[1] << " Z: " << tvec[2];
      std::string text_xyz = ss.str();

      int fontFace = cv::FONT_HERSHEY_SIMPLEX;
      double fontScale = 1.0;
      int thickness = 2;
      int baseline = 0;
      cv::Size textSize = cv::getTextSize(text_xyz, fontFace, fontScale, thickness, &baseline);
      baseline += thickness;
      cv::Point textOrg((result.annotated_image.cols - textSize.width - 10), (result.annotated_image.rows - 10));
      cv::putText(
        result.annotated_image, text_xyz, textOrg, fontFace, fontScale,
        cv::Scalar(0, 255, 255), thickness, 8);
    }

    result.batch.observations.push_back(obs);
    result.total_accepted++;
  }

  return result;
}

}  // namespace full_self_driving::perception
