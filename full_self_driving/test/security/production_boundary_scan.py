#!/usr/bin/env python3

import os
import re
import sys
import unittest

FORBIDDEN_PROTOTYPE_PATTERNS = [
    # Prototype package names, includes, and links
    "px4_roscon_25",
    "aruco_tracker",
    "aruco_database",
    "aruco_database_bridge",
    "find_package(transit_in",
    "find_package(transit_out",
    "find_package(precision_land",
    "find_package(search",
    "<transit_in/",
    "<transit_out/",
    "<precision_land/",
    "<search/",
    "transit_in::",
    "transit_out::",
    "precision_land::",
    # Prototype scripts and launch
    "gazebo_models/run_world.sh",
    "common.launch.py",
]

FORBIDDEN_OFFBOARD_PATTERNS = [
    # Prohibited raw Offboard control symbols and direct topics
    "OffboardControlMode",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
    "/fmu/in/actuator_motors",
    "/fmu/in/actuator_servos",
]

ALLOWED_EXCEPTION_PATHS = [
    "test/fixtures/prototype_behavior_map.yaml",
    "test/launch/launch_manifest_test.cpp",
    "test/security/forbidden_dependency_scan.py",
    "test/security/production_boundary_scan.py",
    "test/property/property_21_ros_interface_boundary.py",
    "simulation/manifests/kmitl_airfield.yaml",
]


class TestProductionBoundaryScan(unittest.TestCase):
    def setUp(self):
        self.package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..")
        )
        self.msg_dir = os.path.join(self.package_root, "msg")
        self.srv_dir = os.path.join(self.package_root, "srv")

    def test_no_forbidden_prototype_or_offboard_in_production(self):
        violations = []
        production_extensions = [".cpp", ".hpp", ".py", ".xml", ".txt", ".urdf", ".yaml", ".json"]
        all_forbidden = FORBIDDEN_PROTOTYPE_PATTERNS + FORBIDDEN_OFFBOARD_PATTERNS

        for root, _, files in os.walk(self.package_root):
            for file in files:
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, self.package_root)
                norm_rel = rel_path.replace("\\", "/")

                # Skip tests, documentation, build artifacts, git, and allowed exception paths
                if (norm_rel.startswith("test/") or
                    norm_rel.startswith("build/") or
                    norm_rel.startswith(".git/") or
                    any(exc in norm_rel for exc in ALLOWED_EXCEPTION_PATHS)):
                    continue

                if not any(file.endswith(ext) for ext in production_extensions):
                    continue

                try:
                    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                        for line_no, line in enumerate(f, 1):
                            for pattern in all_forbidden:
                                if pattern in line:
                                    violations.append(
                                        f"{rel_path}:{line_no} contains forbidden pattern '{pattern}': {line.strip()}"
                                    )
                except Exception as e:
                    violations.append(f"Error reading {rel_path}: {e}")

        if violations:
            msg = "\n".join(["Found forbidden patterns in production code:"] + violations)
            self.fail(msg)

    def test_ros_interfaces_have_strict_bounds(self):
        """Ensure all .msg and .srv fields are bounded (no open-ended strings or unbounded arrays)."""
        violations = []
        interface_dirs = [self.msg_dir, self.srv_dir]

        for d in interface_dirs:
            if not os.path.isdir(d):
                continue
            for file in os.listdir(d):
                if not (file.endswith(".msg") or file.endswith(".srv")):
                    continue
                file_path = os.path.join(d, file)
                rel_path = os.path.relpath(file_path, self.package_root)
                with open(file_path, "r", encoding="utf-8") as f:
                    for line_no, line in enumerate(f, 1):
                        stripped = line.strip()
                        if not stripped or stripped.startswith("#") or "=" in stripped or stripped == "---":
                            continue
                        parts = stripped.split()
                        if not parts:
                            continue
                        type_name = parts[0]

                        # Check unbounded string
                        if type_name == "string":
                            violations.append(
                                f"{rel_path}:{line_no} has unbounded 'string' field: '{stripped}'. Must be string<=N"
                            )
                        # Check unbounded array
                        if type_name.endswith("[]") and not type_name.endswith("[36]"):
                            violations.append(
                                f"{rel_path}:{line_no} has unbounded sequence '{type_name}': '{stripped}'. Must be bounded [<=N]"
                            )

        if violations:
            msg = "\n".join(["Found unbounded fields in ROS interfaces:"] + violations)
            self.fail(msg)

    def test_negative_injection_scanner_detects_violation(self):
        """Verify that the scanner logic correctly detects forbidden patterns when injected."""
        synthetic_bad_line = "find_package(transit_in REQUIRED)"
        detected = any(pat in synthetic_bad_line for pat in FORBIDDEN_PROTOTYPE_PATTERNS)
        self.assertTrue(detected, "Scanner failed to detect synthetic forbidden pattern")

        synthetic_offboard = "msg.topic = /fmu/in/offboard_control_mode"
        detected_offboard = any(pat in synthetic_offboard for pat in FORBIDDEN_OFFBOARD_PATTERNS)
        self.assertTrue(detected_offboard, "Scanner failed to detect synthetic offboard pattern")


if __name__ == "__main__":
    unittest.main()
