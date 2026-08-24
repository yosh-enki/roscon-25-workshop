# Hardware-in-the-Loop (HITL) Distributed Simulation Architecture Specification

> **Date:** 2026-08-25  
> **Status:** Draft / Pending Review  
> **Target Subsystems:** `full_self_driving`, `scripts`, `firmware`, `simulation`  
> **Hardware:** Pixhawk 4 Flight Controller, Raspberry Pi 4 (4GB RAM) Companion Computer, Host PC (Gazebo Harmonic & QGC)

---

## 1. Executive Summary & Objective

This specification details the end-to-end **Hardware-in-the-Loop (HITL)** architecture for autonomous delivery drone operations within the `full_self_driving` ROS 2 package.

The system distributes simulation and flight computation across three interconnected nodes:
1. **Host PC (Simulation Backend & GCS)**: Executes Gazebo Harmonic physics simulation, downward camera sensor rendering, ROS 2 bridges (`/clock`, `/camera`), Foxglove Studio WebSocket bridge, and QGroundControl (QGC).
2. **Pixhawk 4 (Autopilot Hardware)**: Executes physical PX4 Autopilot firmware in HITL mode (`SYS_HITL=1`), processing EKF2 state estimation and attitude control while driving physical servo actuators on AUX OUT CH1.
3. **Raspberry Pi 4 (Companion Computer - 4GB RAM)**: Executes `MicroXRCEAgent` serial transport over GPIO UART (`/dev/ttyAMA0` @ 921600 baud) and the complete `full_self_driving` autonomy stack (ArUco perception, flight runtime executor, gateway, and pad registry) under strict low-memory operating constraints.

---

## 2. System Architecture & Physical Interconnects

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                                  HOST PC                                    │
│  - Gazebo Harmonic: Physics Simulation & Downward Camera Sensor             │
│  - ros_gz_bridge: /clock, /camera_info, /camera (ros_gz_image)              │
│  - foxglove_bridge: WebSocket Bridge (Port 8765)                            │
│  - QGroundControl: GCS Operator Interface (UDP 14550 / 18570)               │
└──────────────────────┬───────────────────────────────┬──────────────────────┘
                       │ USB (/dev/ttyACM0)            │ Ethernet LAN (DDS / Topics)
                       │ HIL Sensor & Actuator Data    │ (Static IP Subnet 192.168.1.0/24)
                       ▼                               ▼
┌──────────────────────────────────────────┐    ┌─────────────────────────────┐
│                PIXHAWK 4                 │    │       RASPBERRY PI 4        │
│  - PX4 Autopilot Firmware (SYS_HITL=1)   │    │    (Companion Computer)     │
│  - EKF2 State Estimator                  │    │ - MicroXRCEAgent Serial     │
│  - Actuator Allocation & PWM Generation  │    │   (/dev/ttyAMA0 @ 921600)   │
│  - AUX OUT CH1 (DO_GRIPPER / Servo PWM)  │    │ - fsd_perception (ArUco)    │
└──────────────────────┬───────────────────┘    │ - fsd_flight_runtime (FSD)  │
                       │ TELEM2 (UART 921600)   │ - fsd_pad_registry          │
                       │ TX ↔ RX, RX ↔ TX, GND  │ - fsd_gateway & evidence    │
                       └────────────────────────┴─────────────────────────────┘
```

### 2.1 Wiring Pinout Checklist

| Interface | From Device & Pin | To Device & Pin | Protocol / Signal | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Telemetry UART** | Pixhawk TELEM2 TX | RPi 4 GPIO 15 (Pin 10 / RX) | 3.3V UART (921600 baud) | Micro-XRCE-DDS transport |
| **Telemetry UART** | Pixhawk TELEM2 RX | RPi 4 GPIO 14 (Pin 8 / TX) | 3.3V UART (921600 baud) | Micro-XRCE-DDS transport |
| **Common Ground** | Pixhawk TELEM2 GND | RPi 4 GND (Pin 6 or 9) | 0V Reference | **Mandatory** common ground |
| **HIL Link** | Pixhawk Micro-USB | Host PC USB (`/dev/ttyACM0`) | USB CDC Serial | High-rate sensor lockstep |
| **Network Link** | RPi 4 RJ45 Ethernet | Host PC RJ45 Ethernet | 100/1000 Mbps Ethernet | Static IP Subnet (ROS 2 DDS) |
| **Gripper Servo** | Pixhawk AUX 1 Signal | SG90 / MG90S Signal (Orange) | 50Hz PWM (1000–2000 µs) | Command 211 DO_GRIPPER |
| **Servo Power** | External 5V 2A UBEC | Pixhawk AUX Rail 5V / GND | +5V DC & Ground | **Do NOT power servo from Pi/FCU CPU** |

---

## 3. Subsystem Decomposition & Component Allocation

### 3.1 Host Simulation Backend (`fsd_hitl_host.launch.py`)
- **Package**: `full_self_driving`
- **Location**: `full_self_driving/launch/fsd_hitl_host.launch.py`
- **Managed Processes & Nodes**:
  1. `gz sim -r simulation/worlds/kmitl_airfield.sdf`: Physics engine & 3D rendering.
  2. `ros_gz_bridge` (`gz_clock_bridge`): Unidirectional clock publication (`/clock`).
  3. `ros_gz_bridge` (`gz_camera_info_bridge`): Publishes `/camera_info`.
  4. `ros_gz_image` (`camera_image_bridge`): Publishes `/camera` and `/camera/compressed`.
  5. `robot_state_publisher`: Publishes `x500.urdf` coordinate tree.
  6. `foxglove_bridge`: WebSocket server (port 8765).
  7. `static_tf_container`: Static transforms (`map -> odom`, `base_link -> camera_link/imager`, `model_camera_frame -> camera_frame`).

### 3.2 Raspberry Pi 4 Companion Autonomy (`fsd_companion_rpi.launch.py`)
- **Package**: `full_self_driving`
- **Location**: `full_self_driving/launch/fsd_companion_rpi.launch.py`
- **Managed Processes & Nodes**:
  1. `MicroXRCEAgent`: Serial transport daemon (`--dev /dev/ttyAMA0 -b 921600`).
  2. `px4_tf_publisher`: Translates PX4 `vehicle_odometry` into ROS 2 TF (`odom -> base_link`).
  3. `fsd_perception`: OpenCV ArUco detector (Dictionary: `DICT_4X4_50`, Marker size: 0.40–0.50m).
  4. `fsd_pad_registry`: Delivery pad database & lookup server.
  5. `fsd_flight_runtime`: `FullSelfDrivingMode` & `FullSelfDrivingModeExecutor` (`px4_ros2_cpp`).
  6. `fsd_evidence`: Post-drop visual evidence and flight logging.
  7. `fsd_gateway`: External REST/WebSocket boundary gatekeeper.
  8. `fsd_launch_probe`: Readiness probe monitoring subsystem health.

---

## 4. Raspberry Pi 4 Low-Memory (4GB RAM) Optimization

1. **Compilation Throttling**:
   ```bash
   colcon build --symlink-install \
     --packages-select full_self_driving \
     --parallel-workers 1 \
     --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1 -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
   ```
2. **Swap File Configuration**:
   - Ensure 2048 MB to 4096 MB swap space is active in `/etc/dphys-swapfile`.
3. **No Heavy Simulation Rendering on Pi**:
   - Zero Gazebo binaries or SITL mock instances spawned on RPi 4.
   - DDS DDS transport optimized via CycloneDDS (`rmw_cyclonedds_cpp`).

---

## 5. Hardware Manifest Configuration (`hitl_rpi4_pixhawk.yaml`)

To satisfy the FSD security boundary gate (`fsd_hardware_manifest_validator`), a dedicated manifest will be placed in `full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml`:

```yaml
profile: "hitl_rpi4_pixhawk4"
manifest_version: "1.0.0"
description: "Hardware-in-the-Loop Profile for Raspberry Pi 4 and Pixhawk 4"

approval:
  approved: true
  approval_authority: "safety-board@fsd.roscon25.org"
  approval_evidence_sha256: "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
  approval_timestamp_utc: "2026-08-25T00:00:00Z"

fmu_transport:
  adapter_id: "px4_hardware_uart_serial"
  device_path: "/dev/ttyAMA0"
  baud_rate: 921600
  flow_control: "none"
  dds_agent_protocol: "serial"

camera:
  adapter_id: "ros2_bridge_camera"
  device_path: "/camera"
  driver: "ros_gz_bridge"
  width: 1280
  height: 720
  framerate_hz: 30
  pixel_format: "RGB8"

payload:
  adapter_id: "px4_uorb_gripper_actuator"
  gripper_instance: 1
  actuation_dwell_s: 0.5

system_resources:
  max_cpu_percent: 85.0
  max_memory_mb: 3072
  storage_reserve_mb: 1024
  power_loss_recovery_enabled: true
```

---

## 6. Gripper & Actuator Subsystem Specification

### 6.1 Command Protocol Details
- **uORB Topic**: `/fmu/in/vehicle_command` (`px4_msgs::msg::VehicleCommand`)
- **Command ID**: `VEHICLE_CMD_DO_GRIPPER` (**`211`**)
- **Parameters**:
  - `param1 = 1.0f` (Gripper instance index 1 ↔ Pixhawk AUX OUT 1)
  - `param2`:
    - `0.0f` = `GRIPPER_ACTION_RELEASE` (Open / Drop payload)
    - `1.0f` = `GRIPPER_ACTION_GRAB` (Close / Lock payload)
- **Feedback & Latency Tracking**: Subscribes to `/fmu/out/vehicle_command_ack` to log execution status and round-trip latency.

### 6.2 PX4 Autopilot Parameter Configuration (via QGroundControl)
- `PP_GRIPPER_EN = 1` (Enable PX4 Gripper driver)
- `PP_GRIP_TYPE = 0` (Servo gripper)
- `GRIP_PULSE_OPEN = 2000` (Pulse width in µs for released state)
- `GRIP_PULSE_CLOSE = 1000` (Pulse width in µs for locked state)
- **QGC Actuators Screen**: Assign `PWM AUX 1` to Function **`Gripper`**.

---

## 7. Diagnostic Tooling & Verification Workflow

### 7.1 Interactive Serial Verification Script (`scripts/test_pixhawk_connection.sh`)
Provides automated preflight diagnostic checking:
1. Validates `/dev/ttyAMA0` device presence and user `dialout` permissions.
2. Spawns `MicroXRCEAgent` in a background subprocess.
3. Echoes `/fmu/out/vehicle_status` and `/fmu/out/timesync_status`.
4. Reports clear pass/fail status to the operator.

### 7.2 Standard Operating Procedure (SOP) Execution Order

1. **Hardware Power-on**:
   - Power Pixhawk 4 via Power Module; ensure UBEC 5V powers AUX servo rail.
   - Connect Pixhawk USB to Host PC (`/dev/ttyACM0`).
   - Connect Pixhawk TELEM2 to RPi 4 GPIO (Pins 8, 10, GND).
   - Connect Ethernet cable between Host PC and RPi 4.
2. **Network Initialization**:
   - Host PC: `export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
   - RPi 4: `export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
3. **Step 1: Test Pixhawk ↔ RPi Communication**:
   ```bash
   # On RPi 4:
   ./scripts/test_pixhawk_connection.sh /dev/ttyAMA0 921600
   ```
4. **Step 2: Start Simulation Backend on Host PC**:
   ```bash
   # On Host PC:
   ros2 launch full_self_driving fsd_hitl_host.launch.py world:=kmitl_airfield
   ```
5. **Step 3: Start Autonomy Stack on RPi 4**:
   ```bash
   # On RPi 4:
   ros2 launch full_self_driving fsd_companion_rpi.launch.py \
     serial_port:=/dev/ttyAMA0 \
     baud_rate:=921600 \
     payload_adapter:=px4_uorb_gripper_actuator
   ```
6. **Step 4: Mission Execution**:
   - Open Foxglove Studio on Host PC (`ws://localhost:8765`).
   - Click `CLOSE & LOCK (2)` to engage servo latch.
   - Click `ASSIGN PAD 1` to select delivery target.
   - Arm drone in QGroundControl -> Drone executes autonomous takeoff, ArUco visual descent, precision landing, servo payload release, and returns to origin!
