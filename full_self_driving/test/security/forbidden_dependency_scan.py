#!/usr/bin/env python3

import os
import sys
import unittest

FORBIDDEN_PATTERNS = [
    # Prototype package names and includes
    "px4_roscon_25",
    "aruco_tracker",
    "aruco_database",
    "aruco_database_bridge",
    "find_package(transit_in)",
    "find_package(transit_out)",
    "find_package(precision_land)",
    "find_package(search)",
    "<transit_in/",
    "<transit_out/",
    "<precision_land/",
    "<search/",
    "transit_in::",
    "transit_out::",
    "precision_land::",
    # Prototype launch and scripts
    "gazebo_models/run_world.sh",
    "common.launch.py",
    # Prohibited Offboard control symbols and direct topics
    "OffboardControlMode",
    "/fmu/in/offboard_control_mode",
    "/fmu/in/trajectory_setpoint",
]

# Paths allowed to contain references for documentation, mapping, or provenance
ALLOWED_EXCEPTION_PATHS = [
    "test/fixtures/prototype_behavior_map.yaml",
    "test/launch/launch_manifest_test.cpp",
    "test/launch/launch_boundary_test.py",
    "test/security/forbidden_dependency_scan.py",
    "test/security/production_boundary_scan.py",
    "simulation/manifests/kmitl_airfield.yaml",
]


class TestForbiddenDependencyScan(unittest.TestCase):
    def setUp(self):
        self.package_root = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..")
        )

    def test_no_forbidden_dependencies_in_production_code(self):
        violations = []
        production_extensions = [".cpp", ".hpp", ".py", ".xml", ".txt", ".urdf", ".yaml", ".json"]

        for root, _, files in os.walk(self.package_root):
            for file in files:
                file_path = os.path.join(root, file)
                rel_path = os.path.relpath(file_path, self.package_root)

                # Skip allowed exception paths and build artifacts
                if any(rel_path == exc or rel_path.startswith("build") or rel_path.startswith(".git") for exc in ALLOWED_EXCEPTION_PATHS):
                    continue

                if not any(file.endswith(ext) for ext in production_extensions):
                    continue

                try:
                    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
                        for line_no, line in enumerate(f, 1):
                            for pattern in FORBIDDEN_PATTERNS:
                                if pattern in line:
                                    violations.append(
                                        f"{rel_path}:{line_no} contains forbidden pattern '{pattern}': {line.strip()}"
                                    )
                except Exception as e:
                    violations.append(f"Error reading {rel_path}: {e}")

        if violations:
            msg = "\n".join(["Found forbidden dependencies in production code:"] + violations)
            self.fail(msg)


if __name__ == "__main__":
    unittest.main()
