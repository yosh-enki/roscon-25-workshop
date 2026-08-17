#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <type_traits>

#include "adapters/px4_api_capabilities.hpp"

using namespace full_self_driving::adapters;

TEST(Px4ApiProbeTest, CompileTimeTypeTraits)
{
  // ModeBase type traits
  static_assert(std::is_base_of_v<px4_ros2::Context, px4_ros2::ModeBase>);
  static_assert(std::is_abstract_v<px4_ros2::ModeBase>);
  static_assert(!std::is_copy_constructible_v<px4_ros2::ModeBase>);

  // ModeExecutorBase type traits
  static_assert(std::is_abstract_v<px4_ros2::ModeExecutorBase>);
  static_assert(!std::is_copy_constructible_v<px4_ros2::ModeExecutorBase>);

  // RequirementFlags & Reporter traits
  static_assert(std::is_default_constructible_v<px4_ros2::RequirementFlags>);
  static_assert(!std::is_abstract_v<px4_ros2::RequirementFlags>);

  // Odometry and State subscription types
  static_assert(std::is_base_of_v<px4_ros2::Subscription<px4_msgs::msg::VehicleLocalPosition>, px4_ros2::OdometryLocalPosition>);
  static_assert(std::is_base_of_v<px4_ros2::Subscription<px4_msgs::msg::VehicleGlobalPosition>, px4_ros2::OdometryGlobalPosition>);
  static_assert(std::is_base_of_v<px4_ros2::Subscription<px4_msgs::msg::HomePosition>, px4_ros2::HomePosition>);
  static_assert(std::is_base_of_v<px4_ros2::Subscription<px4_msgs::msg::VehicleLandDetected>, px4_ros2::LandDetected>);
  static_assert(std::is_base_of_v<px4_ros2::Subscription<px4_msgs::msg::VehicleStatus>, px4_ros2::VehicleStatus>);
  static_assert(std::is_base_of_v<px4_ros2::SetpointBase, px4_ros2::GotoSetpointType>);

  SUCCEED();
}

TEST(Px4ApiProbeTest, MethodPointerSignatures)
{
  // Test member function pointer resolution for ModeBase
  bool (px4_ros2::ModeBase::*do_reg)() = &px4_ros2::ModeBase::doRegister;
  void (px4_ros2::ModeBase::*on_act)() = &px4_ros2::ModeBase::onActivate;
  void (px4_ros2::ModeBase::*on_deact)() = &px4_ros2::ModeBase::onDeactivate;
  void (px4_ros2::ModeBase::*upd_sp)(float) = &px4_ros2::ModeBase::updateSetpoint;
  void (px4_ros2::ModeBase::*chk_arm)(px4_ros2::HealthAndArmingCheckReporter &) = &px4_ros2::ModeBase::checkArmingAndRunConditions;
  void (px4_ros2::ModeBase::*comp)(px4_ros2::Result) = &px4_ros2::ModeBase::completed;
  uint8_t (px4_ros2::ModeBase::*id_fn)() const = &px4_ros2::ModeBase::id;
  bool (px4_ros2::ModeBase::*is_arm)() const = &px4_ros2::ModeBase::isArmed;
  bool (px4_ros2::ModeBase::*is_act)() const = &px4_ros2::ModeBase::isActive;
  px4_ros2::RequirementFlags & (px4_ros2::ModeBase::*reqs)() = &px4_ros2::ModeBase::modeRequirements;

  EXPECT_TRUE(do_reg != nullptr);
  EXPECT_TRUE(on_act != nullptr);
  EXPECT_TRUE(on_deact != nullptr);
  EXPECT_TRUE(upd_sp != nullptr);
  EXPECT_TRUE(chk_arm != nullptr);
  EXPECT_TRUE(comp != nullptr);
  EXPECT_TRUE(id_fn != nullptr);
  EXPECT_TRUE(is_arm != nullptr);
  EXPECT_TRUE(is_act != nullptr);
  EXPECT_TRUE(reqs != nullptr);

  // Test member function pointer resolution for ModeExecutorBase
  bool (px4_ros2::ModeExecutorBase::*exec_do_reg)() = &px4_ros2::ModeExecutorBase::doRegister;
  void (px4_ros2::ModeExecutorBase::*exec_on_act)() = &px4_ros2::ModeExecutorBase::onActivate;
  void (px4_ros2::ModeExecutorBase::*exec_on_deact)(px4_ros2::ModeExecutorBase::DeactivateReason) = &px4_ros2::ModeExecutorBase::onDeactivate;
  void (px4_ros2::ModeExecutorBase::*exec_on_fs_def)() = &px4_ros2::ModeExecutorBase::onFailsafeDeferred;
  void (px4_ros2::ModeExecutorBase::*exec_takeoff)(const px4_ros2::ModeExecutorBase::CompletedCallback &, float, float) = &px4_ros2::ModeExecutorBase::takeoff;
  void (px4_ros2::ModeExecutorBase::*exec_land)(const px4_ros2::ModeExecutorBase::CompletedCallback &) = &px4_ros2::ModeExecutorBase::land;
  void (px4_ros2::ModeExecutorBase::*exec_rtl)(const px4_ros2::ModeExecutorBase::CompletedCallback &) = &px4_ros2::ModeExecutorBase::rtl;
  void (px4_ros2::ModeExecutorBase::*exec_arm)(const px4_ros2::ModeExecutorBase::CompletedCallback &, bool) = &px4_ros2::ModeExecutorBase::arm;
  void (px4_ros2::ModeExecutorBase::*exec_disarm)(const px4_ros2::ModeExecutorBase::CompletedCallback &, bool) = &px4_ros2::ModeExecutorBase::disarm;
  bool (px4_ros2::ModeExecutorBase::*exec_in_charge)() const = &px4_ros2::ModeExecutorBase::isInCharge;
  bool (px4_ros2::ModeExecutorBase::*exec_defer_fs)(bool, int) = &px4_ros2::ModeExecutorBase::deferFailsafesSync;

  EXPECT_TRUE(exec_do_reg != nullptr);
  EXPECT_TRUE(exec_on_act != nullptr);
  EXPECT_TRUE(exec_on_deact != nullptr);
  EXPECT_TRUE(exec_on_fs_def != nullptr);
  EXPECT_TRUE(exec_takeoff != nullptr);
  EXPECT_TRUE(exec_land != nullptr);
  EXPECT_TRUE(exec_rtl != nullptr);
  EXPECT_TRUE(exec_arm != nullptr);
  EXPECT_TRUE(exec_disarm != nullptr);
  EXPECT_TRUE(exec_in_charge != nullptr);
  EXPECT_TRUE(exec_defer_fs != nullptr);
}

TEST(Px4ApiProbeTest, PinnedApiManifestVerification)
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory("full_self_driving");
  std::string manifest_path = pkg_share + "/config/pinned_api_manifest.yaml";

  auto report = Px4ApiCapabilities::verify_api_manifest(manifest_path);
  EXPECT_TRUE(report.is_valid) << "Manifest verification failed with errors:\n" << report.to_string();
  EXPECT_EQ(report.px4_ros2_cpp_version, "0.0.1");
  EXPECT_EQ(report.px4_msgs_version, "2.0.1");
  EXPECT_FALSE(report.verified_headers.empty());
  EXPECT_FALSE(report.verified_classes.empty());
  EXPECT_FALSE(report.verified_messages.empty());
  EXPECT_TRUE(report.errors.empty());
}

TEST(Px4ApiProbeTest, InvalidManifestFailsClosed)
{
  // Test nonexistent path
  auto report1 = Px4ApiCapabilities::verify_api_manifest("/invalid/path/manifest.yaml");
  EXPECT_FALSE(report1.is_valid);
  EXPECT_FALSE(report1.errors.empty());

  // Test empty path
  auto report2 = Px4ApiCapabilities::verify_api_manifest("");
  EXPECT_FALSE(report2.is_valid);
  EXPECT_FALSE(report2.errors.empty());
}
