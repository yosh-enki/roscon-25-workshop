#!/usr/bin/env python3

import os
import unittest
import subprocess
from ament_index_python.packages import get_package_share_directory


class TestLaunchBoundary(unittest.TestCase):
    def setUp(self):
        self.pkg_share = get_package_share_directory("full_self_driving")
        self.launch_file = os.path.join(self.pkg_share, "launch", "full_self_driving.launch.py")

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

    def test_invalid_world_rejection(self):
        """Test that invalid/unlisted world is rejected."""
        cmd = [
            "ros2", "launch", "full_self_driving", "full_self_driving.launch.py",
            "simulation:=true", "world:=non_existent_world"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0, "Launch must fail with unknown world")


if __name__ == "__main__":
    unittest.main()
