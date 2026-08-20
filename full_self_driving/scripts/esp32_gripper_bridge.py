#!/usr/bin/env python3
"""
ESP32 Hardware-in-the-Loop (HITL) Gripper Actuator Bridge Node.

Subscribes to `/fmu/in/vehicle_command` (VEHICLE_CMD_DO_GRIPPER / 211),
translates the command to non-blocking Serial strings sent to ESP32 on /dev/ttyACM0,
and publishes received hardware acknowledgments to `/fmu/out/vehicle_command_ack`.
"""

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    import serial
except ImportError:
    serial = None

from px4_msgs.msg import VehicleCommand, VehicleCommandAck


VEHICLE_CMD_DO_GRIPPER = 211
VEHICLE_CMD_RESULT_ACCEPTED = 0
VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
VEHICLE_CMD_RESULT_DENIED = 2
VEHICLE_CMD_RESULT_UNSUPPORTED = 3
VEHICLE_CMD_RESULT_FAILED = 4


def map_vehicle_command_to_serial_string(command_id: int, param2: float) -> str:
    """Translate PX4 VehicleCommand into ESP32 serial protocol string."""
    if command_id != VEHICLE_CMD_DO_GRIPPER:
        return None
    if abs(param2 - 0.0) < 0.1:  # 0.0 = Release
        return "CMD:RELEASE\n"
    elif abs(param2 - 1.0) < 0.1:  # 1.0 = Grab / Secure
        return "CMD:SECURE\n"
    return None


def parse_serial_ack(line: str):
    """Parse ESP32 serial reply line into (is_ack, result_code)."""
    line = line.strip()
    if line in ("ACK:RELEASED", "ACK:SECURED", "ACK:PONG"):
        return True, VEHICLE_CMD_RESULT_ACCEPTED
    if line.startswith("ERR:"):
        return False, VEHICLE_CMD_RESULT_FAILED
    return False, VEHICLE_CMD_RESULT_UNSUPPORTED


class Esp32GripperBridge(Node):
    def __init__(self):
        super().__init__('esp32_gripper_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('timeout_sec', 0.1)

        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud').get_parameter_value().integer_value

        self.serial_conn = None
        self.running = True
        self.lock = threading.Lock()

        # Connect to serial
        self._init_serial()

        # Best effort QoS matching PX4 DDS conventions
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers & Publishers
        self.sub_cmd = self.create_subscription(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self._on_vehicle_command,
            qos_sub
        )

        self.pub_ack = self.create_publisher(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            qos_pub
        )

        # Background thread to read serial responses without blocking ROS executor
        self.reader_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            f"ESP32 Gripper Bridge initialized on port '{self.port_name}' @ {self.baud_rate} baud."
        )

    def _init_serial(self):
        if serial is None:
            self.get_logger().warn("pyserial is not installed! Serial hardware bridge disabled.")
            return

        candidate_ports = [self.port_name, '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1']
        for p in candidate_ports:
            try:
                conn = serial.Serial(p, self.baud_rate, timeout=0.1)
                conn.dtr = False
                conn.rts = False
                time.sleep(1.5)
                conn.reset_input_buffer()
                self.serial_conn = conn
                self.port_name = p
                self.get_logger().info(f"Successfully opened serial port: {p}")
                return
            except Exception as e:
                self.get_logger().debug(f"Failed to open {p}: {e}")

        self.get_logger().warn(
            f"Could not open any serial port in {candidate_ports}. Bridge will retry in loop."
        )

    def _on_vehicle_command(self, msg: VehicleCommand):
        cmd_str = map_vehicle_command_to_serial_string(msg.command, msg.param2)
        if not cmd_str:
            return

        self.get_logger().info(
            f"Received VehicleCommand {msg.command} (param2={msg.param2}). Forwarding to ESP32: {cmd_str.strip()}"
        )

        with self.lock:
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(cmd_str.encode('utf-8'))
                    self.serial_conn.flush()
                except Exception as e:
                    self.get_logger().error(f"Error writing to serial: {e}")
            else:
                self.get_logger().warn("Serial connection not open; dropping actuation command.")

    def _serial_read_loop(self):
        while self.running and rclpy.ok():
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    raw_line = self.serial_conn.readline()
                    if raw_line:
                        line = raw_line.decode('utf-8', errors='ignore').strip()
                        if line:
                            self.get_logger().info(f"ESP32 Serial Response: '{line}'")
                            is_ack, result_code = parse_serial_ack(line)
                            if is_ack:
                                ack_msg = VehicleCommandAck()
                                ack_msg.timestamp = int(time.time() * 1e6)
                                ack_msg.command = VEHICLE_CMD_DO_GRIPPER
                                ack_msg.result = result_code
                                ack_msg.target_system = 1
                                ack_msg.target_component = 1
                                self.pub_ack.publish(ack_msg)
                                self.get_logger().info(
                                    f"Published VehicleCommandAck(cmd={VEHICLE_CMD_DO_GRIPPER}, result={result_code})"
                                )
                except Exception as e:
                    self.get_logger().error(f"Error reading serial: {e}")
                    time.sleep(0.5)
            else:
                time.sleep(1.0)
                # Attempt reconnect
                self._init_serial()

    def destroy_node(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
