#!/usr/bin/env python3

import os
import unittest
import subprocess
from ament_index_python.packages import get_package_share_directory


class TestLaunchBoundary(unittest.TestCase):
    def setUp(self):
        self.pkg_share = get_package_share_directory("full_self_driving")
        self.launch_dir = os.path.join(self.pkg_share, "launch")
        self.launch_file = os.path.join(self.launch_dir, "full_self_driving.launch.py")

    def test_single_public_launch_entry_point(self):
        """Verify that the clean install tree contains exactly one public launch entry point."""
        self.assertTrue(os.path.isdir(self.launch_dir), f"Launch directory missing: {self.launch_dir}")
        installed_launches = [
            f for f in os.listdir(self.launch_dir)
            if f.endswith(".launch.py") or f.endswith(".launch.xml") or f.endswith(".launch.yaml")
        ]
        self.assertEqual(
            installed_launches,
            ["full_self_driving.launch.py"],
            f"Package must export exactly one launch entry point, found: {installed_launches}",
        )

    def test_hardware_profile_deferral(self):
        """Test that simulation:=false fails with HARDWARE_PROFILE_NOT_CONFIGURED."""
        cmd = [
            "ros2", "launch", "full_self_driving", "full_self_driving.launch.py",
            "simulation:=false"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, "Launch must fail when simulation:=false without hardware manifest")
        combined_output = result.stdout + result.stderr
        self.assertIn("HARDWARE_PROFILE_NOT_CONFIGURED", combined_output)

    def test_hardware_profile_deferral_with_unapproved_manifest(self):
        """Test that simulation:=false with an unapproved manifest fails with HARDWARE_PROFILE_NOT_CONFIGURED."""
        manifest_path = os.path.join(
            self.pkg_share, "test", "fixtures", "manifests", "unapproved_hardware_manifest.yaml"
        )
        if not os.path.exists(manifest_path):
            manifest_path = "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/test/fixtures/manifests/unapproved_hardware_manifest.yaml"
        cmd = [
            "ros2", "launch", "full_self_driving", "full_self_driving.launch.py",
            "simulation:=false", f"hardware_manifest:={manifest_path}"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, "Launch must fail when hardware profile is unapproved")
        combined_output = result.stdout + result.stderr
        self.assertIn("HARDWARE_PROFILE_NOT_CONFIGURED", combined_output)

    def test_hardware_profile_deferral_with_nonexistent_manifest(self):
        """Test that simulation:=false with non-existent manifest fails with HARDWARE_PROFILE_NOT_CONFIGURED."""
        cmd = [
            "ros2", "launch", "full_self_driving", "full_self_driving.launch.py",
            "simulation:=false", "hardware_manifest:=/tmp/non_existent_hw_manifest.yaml"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, "Launch must fail when manifest does not exist")
        combined_output = result.stdout + result.stderr
        self.assertIn("HARDWARE_PROFILE_NOT_CONFIGURED", combined_output)

    def test_invalid_world_rejection(self):
        """Test that invalid/unlisted world is rejected."""
        cmd = [
            "ros2", "launch", "full_self_driving", "full_self_driving.launch.py",
            "simulation:=true", "world:=non_existent_world"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, "Launch must fail with unknown world")

    def test_no_prototype_or_raw_offboard_leak_in_share(self):
        """Ensure no prototype files or raw offboard topics leak into installed package share."""
        forbidden = [
            "px4_roscon_25",
            "find_package(transit_in",
            "aruco_tracker",
            "aruco_database",
            "/fmu/in/offboard_control_mode",
            "OffboardControlMode",
        ]
        allowed_exceptions = ["prototype_behavior_map.yaml", "kmitl_airfield.yaml"]
        for root, _, files in os.walk(self.pkg_share):
            for file in files:
                if any(exc in file for exc in allowed_exceptions) or "test/" in root.replace("\\", "/"):
                    continue
                if file.endswith((".py", ".yaml", ".xml", ".sdf")):
                    full_p = os.path.join(root, file)
                    with open(full_p, "r", encoding="utf-8", errors="ignore") as f:
                        content = f.read()
                        for pat in forbidden:
                            self.assertNotIn(
                                pat,
                                content,
                                f"Installed resource {full_p} contains forbidden prototype pattern '{pat}'",
                            )


if __name__ == "__main__":
    unittest.main()
