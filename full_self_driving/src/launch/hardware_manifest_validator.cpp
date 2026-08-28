#include "hardware_manifest_validator.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <unistd.h>
#include <openssl/evp.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace full_self_driving::launch
{

std::string validation_status_to_string(ValidationStatus status)
{
  switch (status) {
    case ValidationStatus::VALID:
      return "VALID";
    case ValidationStatus::FILE_NOT_FOUND:
      return "FILE_NOT_FOUND";
    case ValidationStatus::SYNTAX_ERROR:
      return "SYNTAX_ERROR";
    case ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD:
      return "SCHEMA_MISSING_REQUIRED_FIELD";
    case ValidationStatus::APPROVAL_DEFERRED_OR_MISSING:
      return "APPROVAL_DEFERRED_OR_MISSING";
    case ValidationStatus::CALIBRATION_CHECKSUM_MISMATCH:
      return "CALIBRATION_CHECKSUM_MISMATCH";
    case ValidationStatus::CALIBRATION_FILE_NOT_FOUND:
      return "CALIBRATION_FILE_NOT_FOUND";
    case ValidationStatus::DEVICE_NODE_UNAVAILABLE:
      return "DEVICE_NODE_UNAVAILABLE";
    case ValidationStatus::SECURITY_KEYSTORE_INVALID:
      return "SECURITY_KEYSTORE_INVALID";
    case ValidationStatus::ADAPTER_ID_MISMATCH:
      return "ADAPTER_ID_MISMATCH";
    case ValidationStatus::UNKNOWN_ERROR:
    default:
      return "UNKNOWN_ERROR";
  }
}

std::string HardwareManifestValidator::compute_file_sha256(const std::string & file_path)
{
  std::ifstream file(file_path, std::ios::binary);
  if (!file.is_open()) {
    return "";
  }

  EVP_MD_CTX * mdctx = EVP_MD_CTX_new();
  if (!mdctx) {
    return "";
  }

  if (EVP_DigestInit_ex(mdctx, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(mdctx);
    return "";
  }

  char buffer[4096];
  while (file.read(buffer, sizeof(buffer)) || file.gcount() > 0) {
    if (EVP_DigestUpdate(mdctx, buffer, file.gcount()) != 1) {
      EVP_MD_CTX_free(mdctx);
      return "";
    }
  }

  unsigned char hash[EVP_MAX_MD_SIZE];
  unsigned int lengthOfHash = 0;
  if (EVP_DigestFinal_ex(mdctx, hash, &lengthOfHash) != 1) {
    EVP_MD_CTX_free(mdctx);
    return "";
  }

  EVP_MD_CTX_free(mdctx);

  std::stringstream ss;
  for (unsigned int i = 0; i < lengthOfHash; ++i) {
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
  }
  return ss.str();
}

ValidationResult HardwareManifestValidator::validate_file(
  const std::string & manifest_path,
  bool check_physical_devices,
  const std::string & base_dir)
{
  ValidationResult result;
  if (!fs::exists(manifest_path)) {
    result.add_violation(
      ValidationStatus::FILE_NOT_FOUND,
      "Manifest file does not exist: " + manifest_path);
    return result;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(manifest_path);
  } catch (const std::exception & e) {
    result.add_violation(
      ValidationStatus::SYNTAX_ERROR,
      std::string("YAML parsing error: ") + e.what());
    return result;
  }

  std::string effective_base = base_dir;
  if (effective_base.empty()) {
    effective_base = fs::path(manifest_path).parent_path().string();
  }

  return validate_yaml(root, check_physical_devices, effective_base);
}

ValidationResult HardwareManifestValidator::validate_yaml(
  const YAML::Node & root,
  bool check_physical_devices,
  const std::string & base_dir)
{
  ValidationResult result;

  if (!root.IsMap()) {
    result.add_violation(ValidationStatus::SYNTAX_ERROR, "Manifest root must be a YAML mapping");
    return result;
  }

  // 1. Check top-level metadata
  if (!root["profile"] || root["profile"].as<std::string>().empty()) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory field: profile");
  }
  if (!root["manifest_version"] || root["manifest_version"].as<std::string>().empty()) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory field: manifest_version");
  }

  // 2. Check approval gate
  if (!root["approval"]) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory field: approval");
  } else {
    const auto & app = root["approval"];
    bool approved = app["approved"] && app["approved"].as<bool>();
    std::string evidence = app["approval_evidence_sha256"] ? app["approval_evidence_sha256"].as<std::string>() : "";
    if (!approved || evidence.empty()) {
      result.add_violation(
        ValidationStatus::APPROVAL_DEFERRED_OR_MISSING,
        "Hardware profile is not approved for production bringup (approval.approved=false or missing evidence hash)");
    }
  }

  // 3. Check FMU Transport
  if (!root["fmu_transport"]) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory section: fmu_transport");
  } else {
    const auto & fmu = root["fmu_transport"];
    std::string adapter_id = fmu["adapter_id"] ? fmu["adapter_id"].as<std::string>() : "";
    if (adapter_id != "px4_hardware_uart_serial") {
      result.add_violation(
        ValidationStatus::ADAPTER_ID_MISMATCH,
        "fmu_transport.adapter_id must be 'px4_hardware_uart_serial', found: '" + adapter_id + "'");
    }
    std::string dev_path = fmu["device_path"] ? fmu["device_path"].as<std::string>() : "";
    if (dev_path.empty()) {
      result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "fmu_transport.device_path must not be empty");
    } else if (check_physical_devices) {
      if (access(dev_path.c_str(), R_OK | W_OK) != 0) {
        result.add_violation(
          ValidationStatus::DEVICE_NODE_UNAVAILABLE,
          "fmu_transport device node inaccessible: " + dev_path);
      }
    }
  }

  // 4. Check Camera
  if (!root["camera"]) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory section: camera");
  } else {
    const auto & cam = root["camera"];
    std::string adapter_id = cam["adapter_id"] ? cam["adapter_id"].as<std::string>() : "";
    if (adapter_id != "v4l2_hardware_camera") {
      result.add_violation(
        ValidationStatus::ADAPTER_ID_MISMATCH,
        "camera.adapter_id must be 'v4l2_hardware_camera', found: '" + adapter_id + "'");
    }
    std::string dev_path = cam["device_path"] ? cam["device_path"].as<std::string>() : "";
    if (dev_path.empty()) {
      result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "camera.device_path must not be empty");
    } else if (check_physical_devices) {
      if (access(dev_path.c_str(), R_OK) != 0) {
        result.add_violation(
          ValidationStatus::DEVICE_NODE_UNAVAILABLE,
          "camera device node inaccessible: " + dev_path);
      }
    }

    std::string calib_file = cam["calibration_file"] ? cam["calibration_file"].as<std::string>() : "";
    std::string expected_sha = cam["calibration_sha256"] ? cam["calibration_sha256"].as<std::string>() : "";
    if (calib_file.empty() || expected_sha.empty()) {
      result.add_violation(
        ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD,
        "camera calibration_file and calibration_sha256 must not be empty");
    } else {
      // Resolve calibration file path
      fs::path resolved_calib_path;
      if (fs::path(calib_file).is_absolute()) {
        if (fs::exists(calib_file)) {
          resolved_calib_path = calib_file;
        }
      } else {
        std::vector<fs::path> search_roots;
        if (!base_dir.empty()) {
          search_roots.push_back(base_dir);
          search_roots.push_back(fs::path(base_dir).parent_path());
          search_roots.push_back(fs::path(base_dir).parent_path().parent_path());
          search_roots.push_back(fs::path(base_dir).parent_path().parent_path().parent_path());
        }
        try {
          search_roots.push_back(ament_index_cpp::get_package_share_directory("full_self_driving"));
        } catch (...) {}
        search_roots.push_back(fs::current_path());

        for (const auto & root_dir : search_roots) {
          fs::path cand = root_dir / calib_file;
          if (fs::exists(cand)) {
            resolved_calib_path = cand;
            break;
          }
        }
      }

      if (resolved_calib_path.empty() || !fs::exists(resolved_calib_path)) {
        result.add_violation(
          ValidationStatus::CALIBRATION_FILE_NOT_FOUND,
          "Calibration file not found for: " + calib_file);
      } else {
        std::string actual_sha = compute_file_sha256(resolved_calib_path.string());
        if (actual_sha != expected_sha) {
          result.add_violation(
            ValidationStatus::CALIBRATION_CHECKSUM_MISMATCH,
            "Calibration SHA-256 mismatch: expected " + expected_sha + ", got " + actual_sha);
        }
      }
    }
  }

  // 5. Check Payload
  if (!root["payload"]) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory section: payload");
  } else {
    const auto & pay = root["payload"];
    std::string adapter_id = pay["adapter_id"] ? pay["adapter_id"].as<std::string>() : "";
    if (adapter_id != "gpio_pwm_payload_actuator" &&
        adapter_id != "px4_uorb_gripper_actuator" &&
        adapter_id != "simulation_payload_stub") {
      result.add_violation(
        ValidationStatus::ADAPTER_ID_MISMATCH,
        "payload.adapter_id must be 'gpio_pwm_payload_actuator', 'px4_uorb_gripper_actuator', or 'simulation_payload_stub', found: '" + adapter_id + "'");
    }
  }

  // 6. Check Security
  if (!root["security"]) {
    result.add_violation(ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD, "Missing mandatory section: security");
  }

  return result;
}

HardwareManifest HardwareManifestValidator::parse_yaml(const YAML::Node & root)
{
  HardwareManifest m;
  if (!root.IsMap()) {
    return m;
  }

  if (root["profile"]) m.profile = root["profile"].as<std::string>();
  if (root["manifest_version"]) m.manifest_version = root["manifest_version"].as<std::string>();
  if (root["description"]) m.description = root["description"].as<std::string>();

  if (root["approval"]) {
    const auto & a = root["approval"];
    if (a["approved"]) m.approval.approved = a["approved"].as<bool>();
    if (a["approval_authority"]) m.approval.approval_authority = a["approval_authority"].as<std::string>();
    if (a["approval_evidence_sha256"]) m.approval.approval_evidence_sha256 = a["approval_evidence_sha256"].as<std::string>();
    if (a["approval_timestamp_utc"]) m.approval.approval_timestamp_utc = a["approval_timestamp_utc"].as<std::string>();
  }

  if (root["fmu_transport"]) {
    const auto & f = root["fmu_transport"];
    if (f["adapter_id"]) m.fmu_transport.adapter_id = f["adapter_id"].as<std::string>();
    if (f["device_path"]) m.fmu_transport.device_path = f["device_path"].as<std::string>();
    if (f["baud_rate"]) m.fmu_transport.baud_rate = f["baud_rate"].as<int>();
    if (f["flow_control"]) m.fmu_transport.flow_control = f["flow_control"].as<std::string>();
    if (f["dds_agent_protocol"]) m.fmu_transport.dds_agent_protocol = f["dds_agent_protocol"].as<std::string>();
  }

  if (root["camera"]) {
    const auto & c = root["camera"];
    if (c["adapter_id"]) m.camera.adapter_id = c["adapter_id"].as<std::string>();
    if (c["device_path"]) m.camera.device_path = c["device_path"].as<std::string>();
    if (c["driver"]) m.camera.driver = c["driver"].as<std::string>();
    if (c["width"]) m.camera.width = c["width"].as<int>();
    if (c["height"]) m.camera.height = c["height"].as<int>();
    if (c["framerate_hz"]) m.camera.framerate_hz = c["framerate_hz"].as<int>();
    if (c["pixel_format"]) m.camera.pixel_format = c["pixel_format"].as<std::string>();
    if (c["calibration_file"]) m.camera.calibration_file = c["calibration_file"].as<std::string>();
    if (c["calibration_sha256"]) m.camera.calibration_sha256 = c["calibration_sha256"].as<std::string>();
  }

  if (root["payload"]) {
    const auto & p = root["payload"];
    if (p["adapter_id"]) m.payload.adapter_id = p["adapter_id"].as<std::string>();
    if (p["device_path"]) m.payload.device_path = p["device_path"].as<std::string>();
    if (p["transport_interface"]) m.payload.transport_interface = p["transport_interface"].as<std::string>();
    if (p["gripper_instance"]) m.payload.gripper_instance = p["gripper_instance"].as<int>();
    if (p["pwm_pin"]) m.payload.pwm_pin = p["pwm_pin"].as<int>();
    if (p["pwm_frequency_hz"]) m.payload.pwm_frequency_hz = p["pwm_frequency_hz"].as<int>();
    if (p["disarmed_pwm_us"]) m.payload.disarmed_pwm_us = p["disarmed_pwm_us"].as<int>();
    if (p["armed_pwm_us"]) m.payload.armed_pwm_us = p["armed_pwm_us"].as<int>();
    if (p["release_pwm_us"]) m.payload.release_pwm_us = p["release_pwm_us"].as<int>();
    if (p["feedback_sense_pin"]) m.payload.feedback_sense_pin = p["feedback_sense_pin"].as<int>();
    if (p["max_pulse_duration_ms"]) m.payload.max_pulse_duration_ms = p["max_pulse_duration_ms"].as<int>();
  }

  if (root["security"]) {
    const auto & s = root["security"];
    if (s["sros2_keystore_path"]) m.security.sros2_keystore_path = s["sros2_keystore_path"].as<std::string>();
    if (s["require_encryption"]) m.security.require_encryption = s["require_encryption"].as<bool>();
    if (s["require_access_control"]) m.security.require_access_control = s["require_access_control"].as<bool>();
  }

  if (root["system_resources"]) {
    const auto & r = root["system_resources"];
    if (r["max_cpu_percent"]) m.system_resources.max_cpu_percent = r["max_cpu_percent"].as<double>();
    if (r["max_memory_mb"]) m.system_resources.max_memory_mb = r["max_memory_mb"].as<int>();
    if (r["storage_reserve_mb"]) m.system_resources.storage_reserve_mb = r["storage_reserve_mb"].as<int>();
    if (r["power_loss_recovery_enabled"]) m.system_resources.power_loss_recovery_enabled = r["power_loss_recovery_enabled"].as<bool>();
  }

  return m;
}

}  // namespace full_self_driving::launch
