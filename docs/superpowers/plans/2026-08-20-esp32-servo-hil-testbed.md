# ESP32 + Servo Hardware-in-the-Loop (HITL) Actuator Testbed Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the physical ESP32 + Servo Motor HITL bridge and firmware to allow real-time physical payload locking and release during ROS 2 / PX4 SITL autonomous delivery sorties.

**Architecture:** A lightweight non-blocking Arduino firmware on ESP32 drives the physical PWM servo on GPIO 18, while a dedicated ROS 2 Python bridge node (`esp32_gripper_bridge.py`) translates between `/fmu/in/vehicle_command` (`VEHICLE_CMD_DO_GRIPPER` / Command ID 211) and the USB Serial link (`/dev/ttyACM0` @ 115200 baud), returning real hardware acknowledgments to `/fmu/out/vehicle_command_ack`.

**Tech Stack:** ROS 2 Humble (`rclpy`, `px4_msgs`), Python 3 (`pyserial`), Arduino / C++ (`ESP32Servo.h`), Docker USB Passthrough.

## Global Constraints
- Target Serial Port: `/dev/ttyACM0` (with fallback to `/dev/ttyUSB0`)
- Baud Rate: `115200`
- Servo PWM Pin: GPIO `18` (50Hz, 500µs to 2400µs pulse range)
- PX4 Command: `px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GRIPPER` (211)
- Property 14 Compliance: Non-blocking, durable intent tracking, idempotent actuation

---

### Task 1: ESP32 Microcontroller Firmware

**Files:**
- Create: `firmware/esp32_gripper_actuator/esp32_gripper_actuator.ino`
- Create: `firmware/esp32_gripper_actuator/README.md`

**Interfaces:**
- Consumes: Serial strings (`CMD:RELEASE\n`, `CMD:SECURE\n`, `CMD:PING\n`) over USB CDC UART @ 115200
- Produces: Serial strings (`ACK:RELEASED\n`, `ACK:SECURED\n`, `ACK:PONG\n`) and hardware PWM pulses on Pin 18

- [ ] **Step 1: Write the ESP32 Arduino sketch**

```cpp
#include <ESP32Servo.h>

Servo myServo;

const int SERVO_PIN = 18;
const int ANGLE_SECURED = 0;    // Locked for flight (0 degrees)
const int ANGLE_RELEASED = 180; // Open for loading/drop (180 degrees)
const int DWELL_TIME_MS = 300;  // Mechanical travel settling time

void setup() {
  Serial.begin(115200);
  
  // Allocate ESP32 PWM Timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(ANGLE_SECURED);
  
  Serial.println("ESP32_GRIPPER_READY");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "CMD:RELEASE") {
      myServo.write(ANGLE_RELEASED);
      delay(DWELL_TIME_MS);
      Serial.println("ACK:RELEASED");
    } else if (cmd == "CMD:SECURE") {
      myServo.write(ANGLE_SECURED);
      delay(DWELL_TIME_MS);
      Serial.println("ACK:SECURED");
    } else if (cmd == "CMD:PING") {
      Serial.println("ACK:PONG");
    } else if (cmd.length() > 0) {
      Serial.print("ERR:UNKNOWN_CMD:");
      Serial.println(cmd);
    }
  }
}
```

- [ ] **Step 2: Write documentation & flashing guide**

Create `firmware/esp32_gripper_actuator/README.md` documenting pinout, baud rate, required library (`ESP32Servo`), and Arduino IDE upload settings.

- [ ] **Step 3: Commit firmware**

```bash
git add firmware/
git commit -m "feat(firmware): add ESP32 native servo gripper firmware for HITL testing"
```

---

### Task 2: ROS 2 Serial Hardware Bridge Node

**Files:**
- Create: `full_self_driving/scripts/esp32_gripper_bridge.py`
- Create: `full_self_driving/test/payload/test_esp32_gripper_bridge.py`
- Modify: `full_self_driving/CMakeLists.txt` (install scripts)

**Interfaces:**
- Consumes: `/fmu/in/vehicle_command` (`px4_msgs.msg.VehicleCommand`)
- Produces: `/fmu/out/vehicle_command_ack` (`px4_msgs.msg.VehicleCommandAck`)
- Hardware I/O: Serial read/write on `/dev/ttyACM0`

- [ ] **Step 1: Write the unit test with mock serial**

Create `full_self_driving/test/payload/test_esp32_gripper_bridge.py` using `unittest` and `unittest.mock` to verify that receiving `VehicleCommand(command=211, param2=0.0)` triggers `CMD:RELEASE\n` over serial, and that serial `ACK:RELEASED` triggers publishing `VehicleCommandAck(result=0)`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest full_self_driving/test/payload/test_esp32_gripper_bridge.py`
Expected: FAIL (module not found).

- [ ] **Step 3: Implement `esp32_gripper_bridge.py`**

Create `full_self_driving/scripts/esp32_gripper_bridge.py` with:
- ROS 2 Node initialization (`esp32_gripper_bridge`)
- Serial connection handler with fallback from `/dev/ttyACM0` to `/dev/ttyUSB0`
- Subscription to `/fmu/in/vehicle_command`
- Publisher on `/fmu/out/vehicle_command_ack`
- Non-blocking serial reader thread

- [ ] **Step 4: Run unit tests and verify they pass**

Run: `pytest full_self_driving/test/payload/test_esp32_gripper_bridge.py`
Expected: PASS.

- [ ] **Step 5: Commit bridge implementation**

```bash
git add full_self_driving/scripts/esp32_gripper_bridge.py full_self_driving/test/payload/test_esp32_gripper_bridge.py
git commit -m "feat(bridge): implement ROS 2 to ESP32 serial gripper actuator bridge"
```

---

### Task 3: Interactive CLI Hardware Smoke Test Script

**Files:**
- Create: `scripts/test_esp32_gripper.py`

**Interfaces:**
- CLI tool to directly send PING, SECURE, and RELEASE commands to `/dev/ttyACM0` and measure roundtrip response latency.

- [ ] **Step 1: Implement `scripts/test_esp32_gripper.py`**

```python
#!/usr/bin/env python3
"""Interactive test script for physical ESP32 Gripper Actuator."""
import sys
import time
import serial

def test_gripper(port="/dev/ttyACM0", baud=115200):
    print(f"Connecting to ESP32 on {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=2.0)
    except Exception as e:
        print(f"Error opening port {port}: {e}")
        return False

    time.sleep(1.5)  # Wait for ESP32 boot
    ser.reset_input_buffer()

    # 1. PING
    print("[1/3] Sending CMD:PING...")
    ser.write(b"CMD:PING\n")
    t0 = time.time()
    resp = ser.readline().decode().strip()
    latency = (time.time() - t0) * 1000
    print(f"      Response: '{resp}' ({latency:.1f} ms)")
    assert "ACK:PONG" in resp, f"Expected ACK:PONG, got {resp}"

    # 2. SECURE
    print("[2/3] Sending CMD:SECURE (0 deg)...")
    ser.write(b"CMD:SECURE\n")
    t0 = time.time()
    resp = ser.readline().decode().strip()
    latency = (time.time() - t0) * 1000
    print(f"      Response: '{resp}' ({latency:.1f} ms)")
    assert "ACK:SECURED" in resp, f"Expected ACK:SECURED, got {resp}"

    time.sleep(1.0)

    # 3. RELEASE
    print("[3/3] Sending CMD:RELEASE (180 deg)...")
    ser.write(b"CMD:RELEASE\n")
    t0 = time.time()
    resp = ser.readline().decode().strip()
    latency = (time.time() - t0) * 1000
    print(f"      Response: '{resp}' ({latency:.1f} ms)")
    assert "ACK:RELEASED" in resp, f"Expected ACK:RELEASED, got {resp}"

    print("\n[SUCCESS] ESP32 Servo Actuator verified successfully!")
    ser.close()
    return True

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    success = test_gripper(port)
    sys.exit(0 if success else 1)
```

- [ ] **Step 2: Commit CLI test tool**

```bash
git add scripts/test_esp32_gripper.py
git commit -m "feat(tools): add interactive ESP32 hardware smoke test script"
```

---

### Task 4: Docker Launch Integration & End-to-End HITL Verification

**Files:**
- Create: `scripts/run_hitl_delivery.sh`
- Modify: `full_self_driving/MANUAL.md` (add Section 13.7 HITL Testbed run guide)

- [ ] **Step 1: Create `scripts/run_hitl_delivery.sh`**

Script configuring Docker container with `--device=/dev/ttyACM0` and launching Gazebo SITL simulation alongside the `esp32_gripper_bridge`.

- [ ] **Step 2: Run all package tests inside Docker**

Verify all 325 unit/property tests pass cleanly inside Docker.

- [ ] **Step 3: Commit integration guide and launch script**

```bash
git add scripts/run_hitl_delivery.sh full_self_driving/MANUAL.md
git commit -m "docs(hitl): add HITL launch script and operational manual"
```
