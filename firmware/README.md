# Firmware & Hardware-in-the-Loop (HITL) Modules

## 📌 Architecture & Clean Separation Guarantee
This directory contains firmware for external hardware testbeds and physical actuator mockups. 

> [!IMPORTANT]
> **Production Isolation Guarantee:**
> * The core flight stack (`full_self_driving.launch.py`, perception pipelines, and flight state machines) is **100% production code** and contains zero testbed or ESP32-specific logic.
> * On real flight hardware, the drone uses Pixhawk's native uORB command bus (`VEHICLE_CMD_DO_GRIPPER`) to drive the physical payload on AUX 1.
> * For desktop development, the ESP32 firmware in this directory acts as a **Hardware-in-the-Loop (HITL) Shadow Actuator**, allowing developers to test physical servo release and lock mechanisms on their desk in sync with Gazebo SITL simulation.

---

## 📂 Subdirectories
* [`esp32_gripper_actuator/`](./esp32_gripper_actuator/): Arduino firmware for ESP32 + Servo Motor HITL Testbed.

---

## ⚡ Quick Start Workflow

### 1. Flash the ESP32
Open [`esp32_gripper_actuator/esp32_gripper_actuator.ino`](./esp32_gripper_actuator/esp32_gripper_actuator.ino) in Arduino IDE:
* Install `ESP32Servo` library.
* Select Board: `ESP32 Dev Module`.
* Port: `/dev/ttyACM0` (or `/dev/ttyUSB0`).
* Click **Upload**.

### 2. Verify Hardware with Smoke Test Script
```bash
cd /home/yosh/roscon-25-workshop
./scripts/test_esp32_gripper.py /dev/ttyACM0
```

### 3. Launch HITL Simulation
```bash
cd /home/yosh/roscon-25-workshop
./scripts/run_hitl_delivery.sh /dev/ttyACM0
```

### 4. Control via Foxglove Studio
Connect Foxglove Studio to `ws://localhost:8766` (or `ws://localhost:8765`), load [`foxglove/roscon-25-workshop.json`](../foxglove/roscon-25-workshop.json), and control the physical servo in real-time.
