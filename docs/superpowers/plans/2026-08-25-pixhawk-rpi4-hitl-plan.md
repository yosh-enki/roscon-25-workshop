# Hardware-in-the-Loop (HITL) Distributed Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and integrate a production-ready, distributed Hardware-in-the-Loop (HITL) workflow supporting Pixhawk 4, Raspberry Pi 4 companion computer (4GB RAM), and Host PC Gazebo Harmonic simulation within the `full_self_driving` ROS 2 package.

**Architecture:** Distributed three-node architecture where Host PC renders physics and downward camera imagery via Gazebo Harmonic; Pixhawk 4 executes PX4 autopilot firmware in HITL mode receiving sensor lockstep via USB and actuating an AUX 1 PWM gripper; and Raspberry Pi 4 runs `MicroXRCEAgent` serial daemon (`/dev/ttyAMA0` @ 921600) and the `full_self_driving` autonomous flight stack over Ethernet LAN.

**Tech Stack:** ROS 2 Humble, PX4 Autopilot v1.14/v1.16, Micro-XRCE-DDS Agent v2.4.2, Gazebo Harmonic, `px4_msgs`, `px4_ros2_cpp`, OpenCV 4.10, Foxglove Bridge, Python 3 / C++17.

---

## Global Constraints

- **Single Launch Entry Points per Node**: Dedicated launch files for Host (`fsd_hitl_host.launch.py`) and Companion (`fsd_companion_rpi.launch.py`).
- **Memory Conservation**: Strict low-memory settings for Raspberry Pi 4 (single worker build, no heavy 3D simulation on Pi).
- **Safety Gate Compatibility**: Hardware manifest compliant with `HardwareManifestValidator` schema and approved state.
- **Actuator Invariance**: Native PX4 gripper execution using `VEHICLE_CMD_DO_GRIPPER` (Command ID 211) on AUX OUT 1.

---

## Task Breakdown

### Task 1: Pixhawk Serial Connectivity & Diagnostic Tooling

**Files:**
- Create: `scripts/test_pixhawk_connection.sh`
- Test: `bash -n scripts/test_pixhawk_connection.sh`

**Interfaces:**
- Consumes: Serial device `/dev/ttyAMA0` (or CLI specified port) @ 921600 baud.
- Produces: Interactive CLI diagnostic script checking serial device existence, `dialout` group membership, spawning `MicroXRCEAgent`, and verifying `/fmu/out/vehicle_status` topic echo.

- [ ] **Step 1: Implement `scripts/test_pixhawk_connection.sh`**
  Write an automated diagnostic script with clear color-coded status checks, auto-detection of device paths, and timeout protection for topic verification.

- [ ] **Step 2: Add executable permissions and test syntax**
  ```bash
  chmod +x scripts/test_pixhawk_connection.sh
  bash -n scripts/test_pixhawk_connection.sh
  ```

- [ ] **Step 3: Commit**
  ```bash
  git add scripts/test_pixhawk_connection.sh
  git commit -m "feat(scripts): add test_pixhawk_connection diagnostic tool for HITL"
  ```

---

### Task 2: HITL Hardware Profile Manifest

**Files:**
- Create: `full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml`
- Test: `test/launch/hardware_manifest_validator_test.cpp`

**Interfaces:**
- Consumes: YAML schema format validated by `full_self_driving/src/launch/hardware_manifest_validator.hpp`.
- Produces: Approved hardware profile defining `/dev/ttyAMA0` @ 921600, `/camera` bridge, and `px4_uorb_gripper_actuator` on instance 1.

- [ ] **Step 1: Create `full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml`**
  Configure approval, `fmu_transport`, `camera`, `payload`, and `system_resources` compliant with FSD safety requirements.

- [ ] **Step 2: Validate manifest with `fsd_hardware_manifest_validator`**
  ```bash
  docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && fsd_hardware_manifest_validator --manifest full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml"
  ```

- [ ] **Step 3: Commit**
  ```bash
  git add full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml
  git commit -m "feat(manifest): add hitl_rpi4_pixhawk hardware profile manifest"
  ```

---

### Task 3: Host Simulation Backend Launch File

**Files:**
- Create: `full_self_driving/launch/fsd_hitl_host.launch.py`
- Create: `full_self_driving/test/launch/test_fsd_hitl_host_launch.py`

**Interfaces:**
- Consumes: `world` (string, default "kmitl_airfield"), `headless` (bool, default false), `foxglove_port` (int, default 8765).
- Produces: Spawns Gazebo Harmonic, `/clock` bridge, `/camera` image & info bridges, `robot_state_publisher`, `foxglove_bridge`, and static TF broadcaster container.

- [ ] **Step 1: Write pytest launch validation test `test_fsd_hitl_host_launch.py`**
  Verify that the launch description includes all required Gazebo, bridge, TF, and Foxglove actions without spawning SITL or companion autonomy nodes.

- [ ] **Step 2: Implement `full_self_driving/launch/fsd_hitl_host.launch.py`**
  Implement the robust launch generator with clean process lifecycle management.

- [ ] **Step 3: Run launch test**
  ```bash
  pytest full_self_driving/test/launch/test_fsd_hitl_host_launch.py
  ```

- [ ] **Step 4: Commit**
  ```bash
  git add full_self_driving/launch/fsd_hitl_host.launch.py full_self_driving/test/launch/test_fsd_hitl_host_launch.py
  git commit -m "feat(launch): add fsd_hitl_host launch entry point for Host PC"
  ```

---

### Task 4: Raspberry Pi Companion Autonomy Launch File

**Files:**
- Create: `full_self_driving/launch/fsd_companion_rpi.launch.py`
- Create: `full_self_driving/test/launch/test_fsd_companion_rpi_launch.py`

**Interfaces:**
- Consumes: `serial_port` (default "/dev/ttyAMA0"), `baud_rate` (default 921600), `payload_adapter` (default "px4_uorb_gripper_actuator"), `hardware_manifest` (path to manifest).
- Produces: Spawns `MicroXRCEAgent` serial transport, `px4_tf_publisher`, `fsd_perception`, `fsd_flight_runtime`, `fsd_pad_registry`, `fsd_evidence`, `fsd_gateway`, and `fsd_launch_probe`.

- [ ] **Step 1: Write pytest launch validation test `test_fsd_companion_rpi_launch.py`**
  Verify that the companion launch description generates all autonomy lifecycle nodes and MicroXRCEAgent serial command while strictly omitting Gazebo and SITL.

- [ ] **Step 2: Implement `full_self_driving/launch/fsd_companion_rpi.launch.py`**
  Implement the companion autonomy launch script.

- [ ] **Step 3: Run launch test**
  ```bash
  pytest full_self_driving/test/launch/test_fsd_companion_rpi_launch.py
  ```

- [ ] **Step 4: Commit**
  ```bash
  git add full_self_driving/launch/fsd_companion_rpi.launch.py full_self_driving/test/launch/test_fsd_companion_rpi_launch.py
  git commit -m "feat(launch): add fsd_companion_rpi launch entry point for Raspberry Pi"
  ```

---

### Task 5: End-to-End Build, Test Verification & Operations Documentation

**Files:**
- Modify: `full_self_driving/docs/08_operations_and_troubleshooting_runbook.md`
- Test: Full package build and test suite execution

**Interfaces:**
- Consumes: All package targets and launch files.
- Produces: 100% passing tests and updated field operations runbook for distributed HITL missions.

- [ ] **Step 1: Execute package build and run test suite**
  ```bash
  colcon build --symlink-install --packages-select full_self_driving --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
  colcon test --packages-select full_self_driving --event-handlers console_direct+
  colcon test-result --all --verbose
  ```

- [ ] **Step 2: Update Runbook `08_operations_and_troubleshooting_runbook.md`**
  Add Section on Distributed HITL (Host + RPi + Pixhawk) operations with exact step-by-step commands.

- [ ] **Step 3: Commit**
  ```bash
  git add full_self_driving/docs/08_operations_and_troubleshooting_runbook.md
  git commit -m "docs(runbook): add distributed HITL operational runbook and commands"
  ```
