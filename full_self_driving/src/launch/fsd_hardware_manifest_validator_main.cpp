#include "hardware_manifest_validator.hpp"

#include <iostream>
#include <string>

using full_self_driving::launch::HardwareManifestValidator;
using full_self_driving::launch::ValidationResult;
using full_self_driving::launch::ValidationStatus;
using full_self_driving::launch::validation_status_to_string;

void print_usage(const char * prog_name)
{
  std::cout << "Usage: " << prog_name << " --manifest <path_to_yaml> [--check-devices] [--json]\n"
            << "Validates hardware deployment manifest against schema, approval gates, and calibration checksums.\n";
}

int main(int argc, char ** argv)
{
  std::string manifest_path;
  bool check_devices = false;
  bool json_output = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--manifest" && i + 1 < argc) {
      manifest_path = argv[++i];
    } else if (arg == "--check-devices") {
      check_devices = true;
    } else if (arg == "--json") {
      json_output = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
  }

  if (manifest_path.empty()) {
    std::cerr << "Error: --manifest argument is required.\n";
    print_usage(argv[0]);
    return 1;
  }

  ValidationResult res = HardwareManifestValidator::validate_file(manifest_path, check_devices);

  if (json_output) {
    std::cout << "{\n"
              << "  \"is_valid\": " << (res.is_valid ? "true" : "false") << ",\n"
              << "  \"status\": \"" << validation_status_to_string(res.status) << "\",\n"
              << "  \"violations\": [\n";
    for (size_t i = 0; i < res.violations.size(); ++i) {
      std::cout << "    \"" << res.violations[i] << "\"" << (i + 1 < res.violations.size() ? "," : "") << "\n";
    }
    std::cout << "  ]\n}\n";
  } else {
    std::cout << "[FSD] Hardware Manifest Validation Result:\n"
              << "  File: " << manifest_path << "\n"
              << "  Valid: " << (res.is_valid ? "YES" : "NO") << "\n"
              << "  Status: " << validation_status_to_string(res.status) << "\n";
    if (!res.is_valid) {
      std::cout << "  Violations (" << res.violations.size() << "):\n";
      for (const auto & v : res.violations) {
        std::cout << "    - " << v << "\n";
      }
    }
  }

  return res.is_valid ? 0 : 2;
}
