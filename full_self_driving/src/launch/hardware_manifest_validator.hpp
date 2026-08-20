#pragma once

#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>

namespace full_self_driving::launch
{

enum class ValidationStatus
{
  VALID,
  FILE_NOT_FOUND,
  SYNTAX_ERROR,
  SCHEMA_MISSING_REQUIRED_FIELD,
  APPROVAL_DEFERRED_OR_MISSING,
  CALIBRATION_CHECKSUM_MISMATCH,
  CALIBRATION_FILE_NOT_FOUND,
  DEVICE_NODE_UNAVAILABLE,
  SECURITY_KEYSTORE_INVALID,
  ADAPTER_ID_MISMATCH,
  UNKNOWN_ERROR
};

std::string validation_status_to_string(ValidationStatus status);

struct ValidationResult
{
  bool is_valid{true};
  ValidationStatus status{ValidationStatus::VALID};
  std::vector<std::string> violations;

  void add_violation(ValidationStatus s, const std::string & msg)
  {
    is_valid = false;
    if (status == ValidationStatus::VALID) {
      status = s;
    }
    violations.push_back(msg);
  }
};

struct HardwareApproval
{
  bool approved{false};
  std::string approval_authority;
  std::string approval_evidence_sha256;
  std::string approval_timestamp_utc;
};

struct FmuTransportConfig
{
  std::string adapter_id;
  std::string device_path;
  int baud_rate{921600};
  std::string flow_control{"none"};
  std::string dds_agent_protocol{"serial"};
};

struct CameraConfig
{
  std::string adapter_id;
  std::string device_path;
  std::string driver{"v4l2"};
  int width{1280};
  int height{720};
  int framerate_hz{30};
  std::string pixel_format{"YUYV"};
  std::string calibration_file;
  std::string calibration_sha256;
};

struct PayloadConfig
{
  std::string adapter_id;
  std::string device_path;
  std::string transport_interface{"vehicle_command"};
  int gripper_instance{1};
  int pwm_pin{18};
  int pwm_frequency_hz{50};
  int disarmed_pwm_us{1000};
  int armed_pwm_us{1500};
  int release_pwm_us{2000};
  int feedback_sense_pin{24};
  int max_pulse_duration_ms{2500};
};

struct SecurityConfig
{
  std::string sros2_keystore_path;
  bool require_encryption{true};
  bool require_access_control{true};
};

struct SystemResourcesConfig
{
  double max_cpu_percent{75.0};
  int max_memory_mb{2048};
  int storage_reserve_mb{1024};
  bool power_loss_recovery_enabled{true};
};

struct HardwareManifest
{
  std::string profile;
  std::string manifest_version;
  std::string description;
  HardwareApproval approval;
  FmuTransportConfig fmu_transport;
  CameraConfig camera;
  PayloadConfig payload;
  SecurityConfig security;
  SystemResourcesConfig system_resources;
};

class HardwareManifestValidator
{
public:
  static ValidationResult validate_file(
    const std::string & manifest_path,
    bool check_physical_devices = false,
    const std::string & base_dir = "");

  static ValidationResult validate_yaml(
    const YAML::Node & root,
    bool check_physical_devices = false,
    const std::string & base_dir = "");

  static HardwareManifest parse_yaml(const YAML::Node & root);

  static std::string compute_file_sha256(const std::string & file_path);
};

}  // namespace full_self_driving::launch
