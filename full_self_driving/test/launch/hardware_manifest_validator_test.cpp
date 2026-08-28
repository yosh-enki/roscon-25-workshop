#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "launch/hardware_manifest_validator.hpp"

#include <filesystem>

namespace fs = std::filesystem;
using namespace full_self_driving::launch;

class HardwareManifestValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::vector<std::string> candidates;
    try {
      candidates.push_back(ament_index_cpp::get_package_share_directory("full_self_driving"));
    } catch (const std::exception &) {}
    candidates.push_back(fs::current_path().string());

    for (const auto & cand : candidates) {
      if (fs::exists(fs::path(cand) / "test" / "fixtures" / "manifests")) {
        pkg_share_dir_ = cand;
        fixtures_dir_ = fs::path(cand) / "test" / "fixtures" / "manifests";
        break;
      }
    }
  }

  std::string pkg_share_dir_;
  fs::path fixtures_dir_;
};

// 1. Valid approved hardware manifest passes validation
TEST_F(HardwareManifestValidatorTest, ValidManifestPasses)
{
  fs::path manifest_path = fixtures_dir_ / "valid_hardware_manifest.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Fixture missing: " << manifest_path;

  // Provide base directory so relative calibration file resolves
  ValidationResult res = HardwareManifestValidator::validate_file(
    manifest_path.string(), false, pkg_share_dir_);

  EXPECT_TRUE(res.is_valid) << "Expected valid manifest, got violations: "
    << (res.violations.empty() ? "none" : res.violations[0]);
  EXPECT_EQ(res.status, ValidationStatus::VALID);
  EXPECT_EQ(res.violations.size(), 0u);
}

// 2. Unapproved hardware manifest fails with APPROVAL_DEFERRED_OR_MISSING
TEST_F(HardwareManifestValidatorTest, UnapprovedManifestFailsClosed)
{
  fs::path manifest_path = fixtures_dir_ / "unapproved_hardware_manifest.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Fixture missing: " << manifest_path;

  ValidationResult res = HardwareManifestValidator::validate_file(
    manifest_path.string(), false, pkg_share_dir_);

  EXPECT_FALSE(res.is_valid);
  EXPECT_EQ(res.status, ValidationStatus::APPROVAL_DEFERRED_OR_MISSING);
  EXPECT_GT(res.violations.size(), 0u);
}

// 3. Tampered calibration hash fails with CALIBRATION_CHECKSUM_MISMATCH
TEST_F(HardwareManifestValidatorTest, TamperedCalibrationFailsChecksum)
{
  fs::path manifest_path = fixtures_dir_ / "tampered_calibration_manifest.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Fixture missing: " << manifest_path;

  ValidationResult res = HardwareManifestValidator::validate_file(
    manifest_path.string(), false, pkg_share_dir_);

  EXPECT_FALSE(res.is_valid);
  EXPECT_EQ(res.status, ValidationStatus::CALIBRATION_CHECKSUM_MISMATCH);
}

// 4. Missing mandatory section fails with SCHEMA_MISSING_REQUIRED_FIELD
TEST_F(HardwareManifestValidatorTest, MissingFmuTransportSectionFails)
{
  fs::path manifest_path = fixtures_dir_ / "missing_fmu_manifest.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Fixture missing: " << manifest_path;

  ValidationResult res = HardwareManifestValidator::validate_file(
    manifest_path.string(), false, pkg_share_dir_);

  EXPECT_FALSE(res.is_valid);
  EXPECT_EQ(res.status, ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD);
}

// 5. Invalid adapter ID fails with ADAPTER_ID_MISMATCH
TEST_F(HardwareManifestValidatorTest, InvalidAdapterIdFails)
{
  fs::path manifest_path = fixtures_dir_ / "invalid_adapter_id_manifest.yaml";
  ASSERT_TRUE(fs::exists(manifest_path)) << "Fixture missing: " << manifest_path;

  ValidationResult res = HardwareManifestValidator::validate_file(
    manifest_path.string(), false, pkg_share_dir_);

  EXPECT_FALSE(res.is_valid);
  EXPECT_EQ(res.status, ValidationStatus::ADAPTER_ID_MISMATCH);
}

// 6. Nonexistent file fails with FILE_NOT_FOUND
TEST_F(HardwareManifestValidatorTest, NonexistentManifestFails)
{
  ValidationResult res = HardwareManifestValidator::validate_file(
    "/non/existent/path/hardware_manifest.yaml", false, pkg_share_dir_);

  EXPECT_FALSE(res.is_valid);
  EXPECT_EQ(res.status, ValidationStatus::FILE_NOT_FOUND);
}

// 7. Test SHA-256 calculation helper directly
TEST_F(HardwareManifestValidatorTest, DirectSha256Calculation)
{
  fs::path calib_path = fs::path(pkg_share_dir_) / "config" / "camera_calibrations" / "imx219_720p.yaml";
  ASSERT_TRUE(fs::exists(calib_path)) << "Calibration file missing: " << calib_path;

  std::string sha = HardwareManifestValidator::compute_file_sha256(calib_path.string());
  EXPECT_FALSE(sha.empty());
  EXPECT_EQ(sha, "c283de9385125caf9014576a6fa7e7e1cd4497a90f3087f4060da9ea71770299");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
