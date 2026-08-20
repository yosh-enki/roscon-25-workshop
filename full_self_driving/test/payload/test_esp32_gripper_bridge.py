#!/usr/bin/env python3
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

# Add scripts dir to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../scripts')))

class TestEsp32GripperBridge(unittest.TestCase):
    def setUp(self):
        # We test message conversion logic and serial command mapping
        pass

    def test_command_translation_release(self):
        from esp32_gripper_bridge import map_vehicle_command_to_serial_string
        # param2 == 0.0 (Release)
        cmd_str = map_vehicle_command_to_serial_string(command_id=211, param2=0.0)
        self.assertEqual(cmd_str, "CMD:RELEASE\n")

    def test_command_translation_secure(self):
        from esp32_gripper_bridge import map_vehicle_command_to_serial_string
        # param2 == 1.0 (Grab / Secure)
        cmd_str = map_vehicle_command_to_serial_string(command_id=211, param2=1.0)
        self.assertEqual(cmd_str, "CMD:SECURE\n")

    def test_command_translation_other_ignored(self):
        from esp32_gripper_bridge import map_vehicle_command_to_serial_string
        # Other commands ignored
        cmd_str = map_vehicle_command_to_serial_string(command_id=999, param2=0.0)
        self.assertIsNone(cmd_str)

    def test_ack_parsing_success(self):
        from esp32_gripper_bridge import parse_serial_ack
        is_ack, result_code = parse_serial_ack("ACK:RELEASED")
        self.assertTrue(is_ack)
        self.assertEqual(result_code, 0)  # VEHICLE_CMD_RESULT_ACCEPTED

        is_ack, result_code = parse_serial_ack("ACK:SECURED")
        self.assertTrue(is_ack)
        self.assertEqual(result_code, 0)

    def test_ack_parsing_error(self):
        from esp32_gripper_bridge import parse_serial_ack
        is_ack, result_code = parse_serial_ack("ERR:UNKNOWN_CMD")
        self.assertFalse(is_ack)

if __name__ == '__main__':
    unittest.main()
