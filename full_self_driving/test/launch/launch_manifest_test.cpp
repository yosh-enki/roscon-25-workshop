#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
      // Fallback for direct testing
      pkg_share_dir_ = "/home/yosh/roscon-25-workshop/full_self_driving";
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
