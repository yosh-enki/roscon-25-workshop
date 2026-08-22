# Module 04: Payload Subsystems & HITL Testbed

The payload management subsystem in `full_self_driving` handles secure cargo latching, delivery actuation, and verification dwells. It features a complete **Hardware Abstraction Layer (HAL)** supporting simulation, native PX4 gripper channels, and a dedicated **ESP32 Hardware-in-the-Loop (HITL) Actuator Testbed**.

---

## 1. Payload Hardware Abstraction Layer (HAL)

All cargo actuators implement the pure virtual interface [`PayloadAdapter`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/payload/payload_adapter.hpp):

```cpp
class PayloadAdapter {
public:
  virtual ~PayloadAdapter() = default;
  virtual std::string get_adapter_id() const = 0;
  virtual bool is_healthy() const = 0;
  virtual bool execute_command(
    uint8_t commanded_state,
    const std::string & operation_id,
    full_self_driving::msg::PayloadStatus & out_status) = 0;
  virtual full_self_driving::msg::PayloadStatus get_status() const = 0;
};
```

```mermaid
graph TD
    PC["PayloadController : PayloadController"]
    ADAPT["PayloadAdapter (Abstract Interface)"]
    
    SIM["SimulationPayloadAdapter (Software Stub)"]
    PX4["Px4GripperPayloadAdapter (uORB VehicleCommand)"]
    HW["HardwarePayloadAdapter / ESP32 Bridge"]
    
    PC --> ADAPT
    ADAPT <|-- SIM
    ADAPT <|-- PX4
    ADAPT <|-- HW
```

### 1.1 `SimulationPayloadAdapter`
- **Adapter ID**: `simulation_payload_stub`
- **Behavior**: Deterministic in-memory simulation for software-only tests and headless CI. Simulates actuator travel delay (default: 500ms) and reports nominal latching states.

### 1.2 `Px4GripperPayloadAdapter`
- **Adapter ID**: `px4_uorb_gripper_actuator`
- **Behavior**: Directly commands PX4 onboard servo/gripper channels by publishing uORB `VehicleCommand` messages (`VEHICLE_CMD_DO_GRIPPER`, command ID 211).
- **Parameters**: `instance` (gripper index 1–8), `commanded_state` (0: Release / Open, 1: Grab / Close & Lock).

### 1.3 `HardwarePayloadAdapter` & ESP32 Serial Gripper
- **Adapter ID**: `gpio_pwm_payload_actuator` / `esp32_serial_bridge`
- **Behavior**: Controls physical servo latches over USB serial communication (`/dev/ttyUSB0` or `/dev/ttyACM0`).

---

## 2. ESP32 Hardware-in-the-Loop (HITL) Testbed

The HITL testbed allows flight software executing in Docker to actuate a real physical servo mechanism on an external microcontroller bench during full simulated flights.

```
┌─────────────────────────────────────────────────────────────┐
│ Docker Simulation Container (Ubuntu 22.04 / ROS 2)           │
│                                                             │
│  fsd_flight_runtime ──> /full_self_driving/payload/prepare  │
│                                │                            │
│                                ▼                            │
│                    esp32_gripper_bridge.py                  │
│                                │ (/dev/ttyUSB0 @ 115200)    │
└────────────────────────────────┼────────────────────────────┘
                                 │ USB CDC Serial
                                 ▼
┌─────────────────────────────────────────────────────────────┐
│ ESP32 Dev Board (esp32_gripper_actuator.ino)                │
│                                                             │
│   JSON Parser ──> 50Hz PWM Generator (LEDC @ GPIO 18)       │
│                                │                            │
│                                ▼                            │
│                      TowerPro SG90 Servo                     │
│                    (0° Open / 90° Closed)                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Microcontroller Firmware (`esp32_gripper_actuator.ino`)

The firmware resides at [`firmware/esp32_gripper_actuator/esp32_gripper_actuator.ino`](file:///home/yosh/roscon-25-workshop/firmware/esp32_gripper_actuator/esp32_gripper_actuator.ino).

### Key Features:
1. **LEDC Hardware PWM**: Configured at 50 Hz (20ms period) with 14-bit resolution (0–16383).
2. **Pulse Width Calibration**:
   - $0^\circ$ (Open / Released): $500 \, \mu\text{s}$ pulse ($\approx 410$ counts)
   - $90^\circ$ (Closed / Locked): $1500 \, \mu\text{s}$ pulse ($\approx 1229$ counts)
   - $180^\circ$ (Maximum): $2500 \, \mu\text{s}$ pulse ($\approx 2048$ counts)
3. **Non-blocking Serial Parser**: Processes newline-terminated JSON commands without delaying control loops.

### Serial Command & Response Protocol

#### Set Servo Angle Command
- **Host $\rightarrow$ ESP32**:
  ```json
  {"cmd": "SET_SERVO", "angle": 90, "op_id": "op_cargo_lock_01"}
  ```
- **ESP32 $\rightarrow$ Host**:
  ```json
  {"status": "OK", "op_id": "op_cargo_lock_01", "angle": 90, "state": "LOCKED"}
  ```

#### Ping / Health Check
- **Host $\rightarrow$ ESP32**:
  ```json
  {"cmd": "PING"}
  ```
- **ESP32 $\rightarrow$ Host**:
  ```json
  {"status": "PONG", "uptime_ms": 14250, "state": "LOCKED", "angle": 90}
  ```

---

## 4. Host Python Bridge (`esp32_gripper_bridge.py`)

The bridge node ([`scripts/esp32_gripper_bridge.py`](file:///home/yosh/roscon-25-workshop/full_self_driving/scripts/esp32_gripper_bridge.py)) provides bidirectional translation between ROS 2 services/topics and the ESP32 serial interface:

### 4.1 Auto-Reset & Bootloader Drain Protection
When opening a serial port on ESP32 boards, the USB-UART chip toggles DTR/RTS lines, causing the microcontroller to reboot and spew ROM bootloader logs (`rst:0x1 (POWERON_RESET)...`).
- The bridge explicitly disables DTR/RTS toggling.
- Upon connection, it flushes and drains the serial buffer for 1.2 seconds, discarding boot noise before sending valid JSON commands.

### 4.2 Graceful Shutdown Handling
Catches SIGINT/SIGTERM cleanly, cancels pending timeouts, and closes serial descriptors without leaving orphaned locks on `/dev/ttyUSB0`.

---

## 5. Hardware Wiring Schematic

| ESP32 Pin | SG90 Servo Wire | Description | Notes |
| :--- | :--- | :--- | :--- |
| **GPIO 18** | Orange / Yellow | PWM Signal Line | 50Hz LEDC Signal |
| **GND** | Brown / Black | Ground | Must share common ground with external 5V power |
| **VIN / Ext 5V** | Red | Power (+5V) | **Do NOT power servo directly from 3.3V pin** |

> [!CAUTION]
> Servos draw inductive peak currents (>500mA) during movement. Always use an external 5V 2A power supply with common ground tied to the ESP32 GND pin to prevent microcontroller brownout resets.

---

## 6. Running HITL Delivery Sortie

The repository includes a single-command automated runner script with NVIDIA GPU passthrough:

```bash
# From workspace root
./scripts/run_hitl_delivery.sh [serial_port]
```

### What the Script Automates:
1. Auto-detects USB serial device (`/dev/ttyUSB0` or `/dev/ttyACM0`) with permissions check.
2. Checks port conflicts on Foxglove WebSocket port (8765) and terminates stale bridge instances.
3. Launches the Docker container with `--device=/dev/ttyUSB0`, `--group-add dialout`, and `--gpus all` (NVIDIA RTX hardware acceleration for Gazebo Harmonic).
4. Brings up Gazebo, PX4 SITL, MicroXRCEAgent, FSD Flight Runtime, and the ESP32 Gripper Bridge simultaneously.
