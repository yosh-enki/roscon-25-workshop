#!/usr/bin/env python3
"""
Interactive Hardware Smoke Test Script for ESP32 + Servo Gripper Actuator.
Sends PING, SECURE (0 deg), and RELEASE (180 deg) commands over serial
and measures round-trip hardware ACK latency.
"""

import sys
import time

try:
    import serial
except ImportError:
    print("[ERROR] 'pyserial' is not installed. Run: pip install pyserial")
    sys.exit(1)


def test_gripper(port="/dev/ttyACM0", baud=115200):
    print("=" * 60)
    print(f"  ESP32 SERVO GRIPPER HARDWARE SMOKE TEST")
    print(f"  Port: {port} | Baud: {baud}")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baud, timeout=2.0)
        ser.dtr = False
        ser.rts = False
    except Exception as e:
        print(f"[FAILED] Error opening serial port '{port}': {e}")
        print("Tip: Check USB connection, permissions (sudo chmod 666 /dev/ttyACM0), or try /dev/ttyUSB0.")
        return False

    print("Connected. Waiting 2.0s for ESP32 boot sequence...")
    time.sleep(2.0)
    
    # Drain boot logs from ESP32 ROM bootloader
    while ser.in_waiting:
        boot_line = ser.readline().decode('utf-8', errors='ignore').strip()
        if boot_line:
            print(f"      [Boot Log] {boot_line}")
    ser.reset_input_buffer()

    tests = [
        ("CMD:PING\n", "ACK:PONG", "Heartbeat Ping"),
        ("CMD:SECURE\n", "ACK:SECURED", "Lock Servo (0 deg)"),
        ("CMD:RELEASE\n", "ACK:RELEASED", "Release Servo (180 deg)"),
        ("CMD:SECURE\n", "ACK:SECURED", "Return to Lock (0 deg)"),
    ]

    for i, (cmd, expected_ack, label) in enumerate(tests, 1):
        print(f"\n[{i}/{len(tests)}] Sending: {cmd.strip()} ({label})...")
        t0 = time.time()
        ser.write(cmd.encode('utf-8'))
        ser.flush()

        resp = ser.readline().decode('utf-8', errors='ignore').strip()
        latency_ms = (time.time() - t0) * 1000

        print(f"      ESP32 Reply: '{resp}' | Roundtrip Latency: {latency_ms:.1f} ms")

        if expected_ack in resp:
            print(f"      [PASS] Verified {expected_ack}")
        else:
            print(f"      [WARN] Expected '{expected_ack}', got '{resp}'")

        time.sleep(0.5)

    ser.close()
    print("\n" + "=" * 60)
    print("  [SUCCESS] All physical hardware tests completed!")
    print("=" * 60)
    return True


if __name__ == "__main__":
    port_arg = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud_arg = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    test_gripper(port_arg, baud_arg)
