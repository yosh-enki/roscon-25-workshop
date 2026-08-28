#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include "launch/hardware_manifest_validator.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;

class LaunchManifestTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    try {
      pkg_share_dir_ = ament_index_cpp::get_package_share_directory("full_self_driving");
    } catch (const std::exception & e) {
      pkg_share_dir_ = fs::current_path().string();
    }
  }

  std::string pkg_share_dir_;
};

TEST_F(LaunchManifestTest, TestSimulationProfileManifestExistsAndValid)
{
  fs::path manifest_path = fs::path(pkg_share_dir_) / "simulation" / "manifests" / "profile_simulation.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Profile simulation manifest must exist at: " << manifest_path;

  std::ifstream f(manifest_path);
  std::stringstream buffer;
  buffer << f.rdbuf();
  std::string content = buffer.str();

  EXPECT_NE(content.find("profile: simulation"), std::string::npos);
  EXPECT_NE(content.find("kmitl_airfield"), std::string::npos);
  EXPECT_NE(content.find("px4_sitl"), std::string::npos);
}

TEST_F(LaunchManifestTest, TestKmitlAirfieldManifestAndProvenance)
{
  fs::path manifest_path = fs::path(pkg_share_dir_) / "simulation" / "manifests" / "kmitl_airfield.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "KMITL airfield manifest must exist at: " << manifest_path;

  std::ifstream f(manifest_path);
  std::stringstream buffer;
  buffer << f.rdbuf();
  std::string content = buffer.str();

  EXPECT_NE(content.find("world_name: \"kmitl_airfield\""), std::string::npos);
  EXPECT_NE(content.find("x500_mono_cam_down_0"), std::string::npos);
  EXPECT_NE(content.find("bd806e45a0f66fc5407889ab2b58cd04674b7b482ff381c44c7245b51f7662d8"), std::string::npos);

  fs::path sdf_path = fs::path(pkg_share_dir_) / "simulation" / "worlds" / "kmitl_airfield.sdf";
  ASSERT_TRUE(fs::exists(sdf_path)) << "World SDF file must exist at: " << sdf_path;
}

TEST_F(LaunchManifestTest, TestHardwareSchemaManifestExistsAndValid)
{
  fs::path schema_path = fs::path(pkg_share_dir_) / "simulation" / "manifests" / "hardware_schema.yaml";
  ASSERT_TRUE(fs::exists(schema_path)) << "Hardware schema manifest must exist at: " << schema_path;

  std::ifstream f(schema_path);
  std::stringstream buffer;
  buffer << f.rdbuf();
  std::string content = buffer.str();

  EXPECT_NE(content.find("profile: \"hardware_rpi4_pixhawk6c\""), std::string::npos);
  EXPECT_NE(content.find("px4_hardware_uart_serial"), std::string::npos);
  EXPECT_NE(content.find("v4l2_hardware_camera"), std::string::npos);
  EXPECT_NE(content.find("gpio_pwm_payload_actuator"), std::string::npos);
}

TEST_F(LaunchManifestTest, TestBridgesAndUrdfExistWithoutPrototypeReferences)
{
  fs::path clock_bridge = fs::path(pkg_share_dir_) / "simulation" / "bridges" / "clock.yaml";
  fs::path camera_bridge = fs::path(pkg_share_dir_) / "simulation" / "bridges" / "camera.yaml";
  fs::path tf_bridge = fs::path(pkg_share_dir_) / "simulation" / "bridges" / "tf.yaml";
  fs::path urdf_file = fs::path(pkg_share_dir_) / "simulation" / "urdf" / "x500.urdf";

  EXPECT_TRUE(fs::exists(clock_bridge));
  EXPECT_TRUE(fs::exists(camera_bridge));
  EXPECT_TRUE(fs::exists(tf_bridge));
  ASSERT_TRUE(fs::exists(urdf_file));

  std::ifstream f(urdf_file);
  std::stringstream buffer;
  buffer << f.rdbuf();
  std::string urdf_content = buffer.str();

  EXPECT_EQ(urdf_content.find("px4_roscon_25"), std::string::npos)
    << "Production URDF must not reference prototype package px4_roscon_25";
}

TEST_F(LaunchManifestTest, AcceptsPx4GripperAdapterInManifest)
{
  std::string yaml_content = R"(
profile: "hardware_rpi4_pixhawk6c_gripper"
manifest_version: "1.0.0"
description: "Test manifest with PX4 native gripper"
approval:
  approved: true
  approval_authority: "safety-board@fsd.roscon25.org"
  approval_evidence_sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  approval_timestamp_utc: "2026-08-20T00:00:00Z"
fmu_transport:
  adapter_id: "px4_hardware_uart_serial"
  device_path: "/dev/null"
camera:
  adapter_id: "v4l2_hardware_camera"
  device_path: "/dev/null"
  calibration_file: "config/camera_calibrations/imx219_720p.yaml"
  calibration_sha256: "c283de9385125caf9014576a6fa7e7e1cd4497a90f3087f4060da9ea71770299"
payload:
  adapter_id: "px4_uorb_gripper_actuator"
  transport_interface: "vehicle_command"
  gripper_instance: 1
security:
  sros2_keystore_path: "/tmp"
  require_encryption: false
  require_access_control: false
system_resources:
  max_cpu_percent: 80.0
  max_memory_mb: 2048
  storage_reserve_mb: 1024
  power_loss_recovery_enabled: true
)";

  YAML::Node root = YAML::Load(yaml_content);
  auto result = full_self_driving::launch::HardwareManifestValidator::validate_yaml(root, false, pkg_share_dir_);
  EXPECT_TRUE(result.is_valid) << (result.violations.empty() ? "" : result.violations[0]);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
