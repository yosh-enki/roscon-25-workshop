#include <filesystem>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "perception/aruco_detector.hpp"

namespace full_self_driving::perception::test
{

class ArucoReplayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    package_share_ = ament_index_cpp::get_package_share_directory("full_self_driving");
    
    // Check installed share location first
    if (std::filesystem::exists(package_share_ + "/test/fixtures/prototype_behavior/aruco/camera_info.yaml")) {
      fixture_dir_ = package_share_ + "/test/fixtures/prototype_behavior/aruco";
    } else if (std::filesystem::exists(package_share_ + "/../../src/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior/aruco/camera_info.yaml")) {
      fixture_dir_ = package_share_ + "/../../src/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior/aruco";
    } else {
      fixture_dir_ = "test/fixtures/prototype_behavior/aruco";
    }

    load_camera_info();
  }

  void load_camera_info()
  {
    std::string cam_info_file = fixture_dir_ + "/camera_info.yaml";
    YAML::Node cam_node = YAML::LoadFile(cam_info_file);

    camera_info_.width = cam_node["width"].as<uint32_t>();
    camera_info_.height = cam_node["height"].as<uint32_t>();
    camera_info_.distortion_model = cam_node["distortion_model"].as<std::string>();

    camera_info_.d.clear();
    for (const auto & val : cam_node["d"]) {
      camera_info_.d.push_back(val.as<double>());
    }

    auto k_node = cam_node["k"];
    for (size_t i = 0; i < 9 && i < k_node.size(); ++i) {
      camera_info_.k[i] = k_node[i].as<double>();
    }

    auto r_node = cam_node["r"];
    for (size_t i = 0; i < 9 && i < r_node.size(); ++i) {
      camera_info_.r[i] = r_node[i].as<double>();
    }

    auto p_node = cam_node["p"];
    for (size_t i = 0; i < 12 && i < p_node.size(); ++i) {
      camera_info_.p[i] = p_node[i].as<double>();
    }
  }

  std::string package_share_;
  std::string fixture_dir_;
  sensor_msgs::msg::CameraInfo camera_info_;
};

TEST_F(ArucoReplayTest, TestSingleMarkerDetectionAndPoseParity)
{
  DetectorConfig config;
  config.dictionary_name = "DICT_4X4_50";
  config.marker_size_m = 0.4;
  config.camera_frame = "camera_frame";
  config.map_id = "kmitl_airfield";
  config.scenario_id = "default_scenario";
  config.target_namespace = "aavc2026";

  ArucoDetector detector(config);
  ASSERT_TRUE(detector.update_camera_info(camera_info_));
  ASSERT_TRUE(detector.is_calibrated());
  ASSERT_EQ(detector.get_calibration_hash().length(), 64UL);

  std::string image_path = fixture_dir_ + "/single_marker_id1.png";
  cv::Mat img = cv::imread(image_path);
  ASSERT_FALSE(img.empty()) << "Failed to read image: " << image_path;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  stamp.nanosec = 500000;

  DetectionResult result = detector.process_image(img, stamp, 1000000ULL, 1);

  EXPECT_EQ(result.total_detected, 1UL);
  EXPECT_EQ(result.total_accepted, 1UL);
  ASSERT_EQ(result.batch.observations.size(), 1UL);

  const auto & obs = result.batch.observations[0];
  EXPECT_EQ(obs.identity.marker_id, 1U);
  EXPECT_EQ(obs.identity.dictionary, "DICT_4X4_50");
  EXPECT_EQ(obs.identity.target_namespace, "aavc2026");
  EXPECT_EQ(obs.map_id, "kmitl_airfield");
  EXPECT_EQ(obs.scenario_id, "default_scenario");
  EXPECT_EQ(obs.pose_frame, "camera_frame");
  EXPECT_EQ(obs.observation_state, full_self_driving::msg::AllIdObservation::QUALITY_ACCEPTED);

  // Load golden reference
  std::string golden_file = fixture_dir_ + "/golden_observations.yaml";
  YAML::Node golden_node = YAML::LoadFile(golden_file);
  auto single_golden = golden_node["single_marker_id1"];

  double expected_z = single_golden["tvec"][2].as<double>();
  double tol_pos = single_golden["tolerance"]["position_m"].as<double>();

  EXPECT_NEAR(obs.pose.position.x, single_golden["tvec"][0].as<double>(), tol_pos);
  EXPECT_NEAR(obs.pose.position.y, single_golden["tvec"][1].as<double>(), tol_pos);
  EXPECT_NEAR(obs.pose.position.z, expected_z, tol_pos);

  // Quaternion normalization check
  double q_norm = std::sqrt(
    obs.pose.orientation.x * obs.pose.orientation.x +
    obs.pose.orientation.y * obs.pose.orientation.y +
    obs.pose.orientation.z * obs.pose.orientation.z +
    obs.pose.orientation.w * obs.pose.orientation.w);
  EXPECT_NEAR(q_norm, 1.0, 1e-4);

  // Quality check
  EXPECT_GT(obs.quality, 0.0f);
  EXPECT_LE(obs.quality, 1.0f);

  // Covariance check: finite positive diagonal
  EXPECT_GT(obs.covariance[0], 0.0);
  EXPECT_GT(obs.covariance[7], 0.0);
  EXPECT_GT(obs.covariance[14], 0.0);
  EXPECT_GT(obs.covariance[21], 0.0);
  EXPECT_GT(obs.covariance[28], 0.0);
  EXPECT_GT(obs.covariance[35], 0.0);

  // Annotated image check
  EXPECT_FALSE(result.annotated_image.empty());
  EXPECT_EQ(result.annotated_image.cols, img.cols);
  EXPECT_EQ(result.annotated_image.rows, img.rows);
}

TEST_F(ArucoReplayTest, TestMultiMarkerBatchPreservesAllIds)
{
  DetectorConfig config;
  config.dictionary_name = "DICT_4X4_50";
  config.marker_size_m = 0.4;

  ArucoDetector detector(config);
  ASSERT_TRUE(detector.update_camera_info(camera_info_));

  std::string image_path = fixture_dir_ + "/multi_marker_id1_id2.png";
  cv::Mat img = cv::imread(image_path);
  ASSERT_FALSE(img.empty());

  builtin_interfaces::msg::Time stamp;
  DetectionResult result = detector.process_image(img, stamp, 2000000ULL, 2);

  EXPECT_EQ(result.total_detected, 2UL);
  EXPECT_EQ(result.total_accepted, 2UL);
  ASSERT_EQ(result.batch.observations.size(), 2UL);

  std::set<uint32_t> detected_ids;
  for (const auto & obs : result.batch.observations) {
    detected_ids.insert(obs.identity.marker_id);
    EXPECT_TRUE(std::isfinite(obs.pose.position.x));
    EXPECT_TRUE(std::isfinite(obs.pose.position.y));
    EXPECT_TRUE(std::isfinite(obs.pose.position.z));
  }

  EXPECT_EQ(detected_ids.count(1), 1UL);
  EXPECT_EQ(detected_ids.count(2), 1UL);
}

TEST_F(ArucoReplayTest, TestBlankImageProducesNoDetections)
{
  DetectorConfig config;
  ArucoDetector detector(config);
  ASSERT_TRUE(detector.update_camera_info(camera_info_));

  std::string image_path = fixture_dir_ + "/blank.png";
  cv::Mat img = cv::imread(image_path);
  ASSERT_FALSE(img.empty());

  builtin_interfaces::msg::Time stamp;
  DetectionResult result = detector.process_image(img, stamp, 3000000ULL, 3);

  EXPECT_EQ(result.total_detected, 0UL);
  EXPECT_EQ(result.total_accepted, 0UL);
  EXPECT_TRUE(result.batch.observations.empty());
}

TEST_F(ArucoReplayTest, TestMissingOrInvalidCalibrationFailsClosed)
{
  DetectorConfig config;
  ArucoDetector detector(config);

  EXPECT_FALSE(detector.is_calibrated());
  EXPECT_TRUE(detector.get_calibration_hash().empty());

  std::string image_path = fixture_dir_ + "/single_marker_id1.png";
  cv::Mat img = cv::imread(image_path);
  ASSERT_FALSE(img.empty());

  builtin_interfaces::msg::Time stamp;
  DetectionResult result = detector.process_image(img, stamp, 4000000ULL, 4);

  EXPECT_FALSE(result.calibration_valid);
  EXPECT_TRUE(result.batch.observations.empty());

  // Pass invalid camera info (zero focal length)
  sensor_msgs::msg::CameraInfo invalid_cam = camera_info_;
  invalid_cam.k[0] = 0.0;
  EXPECT_FALSE(detector.update_camera_info(invalid_cam));
  EXPECT_FALSE(detector.is_calibrated());
}

TEST_F(ArucoReplayTest, TestDictionaryNameAndIdConversions)
{
  EXPECT_EQ(ArucoDetector::dictionary_name_to_id("DICT_4X4_50"), cv::aruco::DICT_4X4_50);
  EXPECT_EQ(ArucoDetector::dictionary_name_to_id("DICT_4X4_250"), cv::aruco::DICT_4X4_250);
  EXPECT_EQ(ArucoDetector::dictionary_name_to_id("DICT_5X5_1000"), cv::aruco::DICT_5X5_1000);
  EXPECT_EQ(ArucoDetector::dictionary_name_to_id("DICT_6X6_250"), cv::aruco::DICT_6X6_250);
  EXPECT_EQ(ArucoDetector::dictionary_name_to_id("DICT_ARUCO_ORIGINAL"), cv::aruco::DICT_ARUCO_ORIGINAL);

  EXPECT_EQ(ArucoDetector::dictionary_id_to_name(cv::aruco::DICT_4X4_250), "DICT_4X4_250");
  EXPECT_EQ(ArucoDetector::dictionary_id_to_name(cv::aruco::DICT_6X6_250), "DICT_6X6_250");
}

TEST_F(ArucoReplayTest, TestCalibrationHashConsistency)
{
  std::string hash1 = ArucoDetector::compute_calibration_hash(camera_info_);
  std::string hash2 = ArucoDetector::compute_calibration_hash(camera_info_);
  EXPECT_EQ(hash1, hash2);
  EXPECT_EQ(hash1.length(), 64UL);

  sensor_msgs::msg::CameraInfo modified_info = camera_info_;
  modified_info.k[0] += 1.0;
  std::string hash3 = ArucoDetector::compute_calibration_hash(modified_info);
  EXPECT_NE(hash1, hash3);
}

}  // namespace full_self_driving::perception::test
