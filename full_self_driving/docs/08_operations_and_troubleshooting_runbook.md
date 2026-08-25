# Module 08: Operations & Troubleshooting Runbook

This runbook provides step-by-step Standard Operating Procedures (SOP), command references, pre-flight checklists, and a comprehensive field troubleshooting matrix for the `full_self_driving` system.

---

## 1. Standard Operating Procedures (SOP)

### 1.1 Environment Setup (Container)
Inside the development Docker container:
```bash
# Source underlay and overlay workspaces
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# Build package (Optimized for low-memory environments)
cd /home/ubuntu/roscon-25-workshop_ws
colcon build --symlink-install \
  --packages-select full_self_driving \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=2 -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
source install/setup.bash
```

---

### 1.2 Launch Modes Catalog

#### Mode A: Full Simulation with Gazebo GUI
Starts Gazebo Harmonic, PX4 SITL, MicroXRCEAgent, Foxglove WebSocket bridge, and all autonomy nodes:
```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=false
```

#### Mode B: Headless Simulation (CI / Automated Testing)
Runs without rendering Gazebo GUI, consuming minimal CPU and RAM:
```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=true
```

#### Mode E: Distributed Hardware-in-the-Loop (Host PC + RPi 4 + Pixhawk 4)
Executes physics & sensor rendering on Host PC while running companion autonomy stack on Raspberry Pi 4:
```bash
# 1. On Raspberry Pi 4 (Test Serial Connection):
./scripts/test_pixhawk_connection.sh /dev/ttyAMA0 921600

# 2. On Host PC (Simulation Backend & Bridges):
ros2 launch full_self_driving fsd_hitl_host.launch.py world:=kmitl_airfield

# 3. On Raspberry Pi 4 (Companion Autonomy Stack):
ros2 launch full_self_driving fsd_companion_rpi.launch.py \
  serial_port:=/dev/ttyAMA0 \
  baud_rate:=921600 \
  payload_adapter:=px4_uorb_gripper_actuator
```

---


## 2. Step-by-Step Mission Execution Workflow

```mermaid
graph TD
    A[1. Launch System via Single Command] --> B[2. Open Foxglove Studio & Connect ws://localhost:8765]
    B --> C[3. Verify Readiness: All Nodes Ready in Foxglove HUD]
    C --> D[4. Close & Lock Payload Cargo: Click 'CLOSE & LOCK']
    D --> E[5. Assign Target Pad: Select Pad 1..6 & Click 'ASSIGN PAD']
    E --> F[6. Arm Vehicle: QGroundControl / Autopilot Arm Trigger]
    F --> G[7. Autonomous Sortie: Takeoff -> TransitIn -> Direct/Search -> PrecisionLand]
    G --> H[8. Automatic Touchdown & Payload Release on Ground]
    H --> I[9. Climb & Return: TakeoffAfterDelivery -> TransitOut -> RTL]
    I --> J[10. Touchdown at Home Base & Multi-Sortie Reset]
```

### Phase 1: Pre-Flight Verification
1. Verify ROS 2 topic activity:
   ```bash
   ros2 topic echo /clock --once
   ros2 topic echo /full_self_driving/health --once
   ros2 topic echo /full_self_driving/readiness --once
   ```
2. Confirm readiness report returns `is_ready: true`:
   - `failure_codes` must be empty.

### Phase 2: Ground Mission Preparation
1. **Secure Cargo**: On the Foxglove HUD left panel, click **`CLOSE & LOCK`** (or call `/full_self_driving/payload/prepare` with `state: 1`).
2. **Assign Target**: Click **`Pad 1`** (Marker ID 1) $\rightarrow$ Click **`ASSIGN PAD 1`** (or call `/full_self_driving/target/select` with `marker_id: 1`).

### Phase 3: Autonomous Sortie Execution
1. Trigger Arming in QGroundControl or via PX4 Arm command.
2. The `FullSelfDrivingModeExecutor` automatically takes over, climbs to 10m AGL, navigates ingress waypoints, acquires Pad 1, executes precision touchdown, and unlatches the payload.
3. The drone climbs to 15m, navigates egress waypoints, and returns to the locked home base origin.

---

## 3. Automated Verification & Acceptance Tests

### 3.1 Run Full Automated Test Suite (All Unit & Property Tests)
```bash
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose
```

### 3.2 Run Full Autonomous Sortie Acceptance Test
```bash
python3 /home/yosh/roscon-25-workshop/full_self_driving/test/acceptance/full_sortie_acceptance.py
```
- Validates the end-to-end mission lifecycle without manual GUI interaction.

---

## 4. Field Troubleshooting Matrix

### 4.1 General Autonomy & Systems Matrix

| Symptom | Probable Root Cause | Diagnostic Command | Remediation Step |
| :--- | :--- | :--- | :--- |
| **Drone refuses to arm; takeoff aborted** | Target identity uncommitted or payload unsecured | `ros2 topic echo /full_self_driving/readiness --once` | Select a target pad and click `CLOSE & LOCK` in Foxglove before arming. |
| **Camera topic empty (`/camera`)** | Image bridge disconnected or Gazebo model name mismatch | `ros2 topic list \| grep camera` | Verify Gazebo camera sensor is active: `gz topic -l \| grep image`. |
| **Foxglove Studio cannot connect (`ws://localhost:8765`)** | Port 8765 in use or stale foxglove bridge running | `lsof -i :8765` or `netstat -tuln \| grep 8765` | Kill stale process `kill -9 $(lsof -t -i:8765)` or specify `foxglove_port:=8766`. |
| **ESP32 Servo not moving during HITL** | USB port permissions or baud rate mismatch | `ls -l /dev/ttyUSB*` and check user in `dialout` | Add user to dialout: `sudo usermod -a -G dialout $USER`. Run interactive test: `python3 scripts/test_esp32_gripper.py`. |
| **Drone stuck in `HOLD` mode** | Manual takeover was triggered by RC stick jitter | `ros2 topic echo /full_self_driving/safety --once` | Reset takeover state via `/full_self_driving/flight/reset_takeover` service or Foxglove HUD. |
| **`HARDWARE_PROFILE_NOT_CONFIGURED` error** | Launched with `simulation:=false` without approved manifest | Check launch arguments | Pass `--hardware_manifest /path/to/approved_manifest.yaml` or use `simulation:=true`. |
| **ArUco markers not detected at 15m altitude** | Camera out of focus or marker size mismatch | `ros2 param get /fsd_perception marker_size` | Verify `marker_size_m` in `fsd_parameters.yaml` matches physical pad size (0.4m or 0.5m). |
| **RTL returns to delivery pad instead of home base** | Sortie origin coordinate not locked upon initial takeoff | Check `flight_runtime_node` log for `[ORIGIN]` | Ensure you are on latest firmware commit with `lock sortie origin` fix (`f15551e`). |

---

### 4.2 Pixhawk ↔ Raspberry Pi 4 Serial & uXRCE-DDS Troubleshooting SOP

When deploying the companion container via `./run_raspi.sh`, use the following decision matrix to troubleshoot hardware link failures:

```mermaid
graph TD
    A[Agent stuck on 'running... | fd: 3'] --> B[Stop container & run: sudo xxd -c 16 /dev/ttyAMA0]
    B -->|Stream of 0xfe bytes| C[MAVLink port collision on TELEM2]
    B -->|Empty stream / No bytes| D[Check uxrce_dds_client status in NSH]
    B -->|XRCE framing bytes present| E[Check Baud rate mismatch]
    
    C --> C1[Set MAV_x_CONFIG=0 & UXRCE_DDS_CFG=102 in QGC -> Reboot FCU]
    D --> D1[Verify Board Architecture: 'ver all' in NSH]
    D1 -->|PX4_FMU_V6C Pixhawk 6C| D2[TELEM2 is /dev/ttyS3: uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600]
    D1 -->|PX4_FMU_V5 Pixhawk 4| D3[TELEM2 is /dev/ttyS2: uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600]
```

| Symptom | Root Cause | Verification Command | Permanent Solution |
| :--- | :--- | :--- | :--- |
| **`MicroXRCEAgent` stuck on `running... \| fd: 3` with no `/fmu` topics** | Port conflict: MAVLink daemon is actively broadcasting on TELEM2 instead of uXRCE-DDS. | `sudo xxd -c 16 /dev/ttyAMA0`<br>*(Outputs packets beginning with `0xfe`)* | 1. In QGC, set `MAV_1_CONFIG` / `MAV_2_CONFIG` to `Disabled` (0).<br>2. Set `UXRCE_DDS_CFG = 102` (TELEM2) and `SER_TEL2_BAUD = 921600`.<br>3. Reboot Flight Controller. |
| **NSH reports `Running, disconnected`** | PX4 client mapped to incorrect hardware UART device for the board target. | In QGC MAVLink Console (`nsh>`):<br>`ver all`<br>`uxrce_dds_client status` | **Identify exact board target:**<br>• **Pixhawk 6C (`PX4_FMU_V6C`)**: TELEM2 is **`/dev/ttyS3`** (UART5)<br>• **Pixhawk 4 (`PX4_FMU_V5`)**: TELEM2 is **`/dev/ttyS2`** (UART2)<br>• **Cube Orange**: TELEM2 is **`/dev/ttyS1`**<br>Manual test: `uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600` |
| **`Running, disconnected` after agent restart** | PX4 backoff retry bug: Client went into dormant retry state before agent started. | `uxrce_dds_client status` | Keep `./run_raspi.sh logs` running on RPi, then in NSH run:<br>`uxrce_dds_client stop`<br>`uxrce_dds_client start -t serial -d <DEVICE> -b 921600`<br>*(or hard power cycle Pixhawk while Agent is running)* |
| **No serial device `/dev/ttyAMA0` on Raspberry Pi** | Serial interface disabled in Raspberry Pi firmware config. | `ls -l /dev/ttyAMA0` | Enable in `/boot/firmware/config.txt`:<br>`enable_uart=1`<br>`dtoverlay=disable-bt`<br>Disable Linux Serial Login Shell via `sudo raspi-config`. |

