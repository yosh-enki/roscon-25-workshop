# Full Self Driving (FSD) Production Refactor & Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Clean up all dead code, purge legacy prototype packages, isolate `px4_tf`, remove hardcoded personal paths, and synchronize flight configurations to make the Full Self Driving stack 100% production-ready for SITL (`full_self_driving.launch.py`) and Real Flight (`fsd_real_flight.launch.py`) controlled via Foxglove Studio.

**Architecture:** Standalone ROS 2 C++ Full Self Driving architecture. Primary nodes (`fsd_flight_runtime`, `fsd_perception`, `fsd_pad_registry`, `fsd_gateway`, `fsd_evidence`, `px4_tf_publisher`) run with isolated callback groups, strict lifecycle management, zero dependency on legacy prototype packages, and standard `ament_index_cpp` resource discovery.

**Tech Stack:** ROS 2 Humble, PX4 Autopilot v1.14 / ROS 2 Bridge (`px4_ros2_cpp`, `px4_msgs`), OpenCV 4, ArUco, Foxglove Studio (`foxglove_bridge`, TypeScript extension), C++17, Python 3.10 / 3.12.

**Launch & Operation Targets:**
1. **SITL Simulation:** `ros2 launch full_self_driving full_self_driving.launch.py`
2. **Real Hardware Flight (RPi 4 + Pixhawk):** `./run_raspi.sh` -> `ros2 launch full_self_driving fsd_real_flight.launch.py`
3. **GCS Mission Control:** Foxglove Studio via WebSocket (`ws://<IP>:8765`) using layout `foxglove/fsd_management.json`

---

## Global Constraints

- **Preserve Foxglove Interface:** Do not break or alter topic names (`/full_self_driving/state`, `/full_self_driving/readiness`, `/full_self_driving/payload/status`, `/full_self_driving/perception/live_target_lock`, `/full_self_driving/telemetry`, `/full_self_driving/pad_registry`, `/full_self_driving/working_plan/status`) or service names (`/full_self_driving/select_target`, `/full_self_driving/prepare_payload`, `/full_self_driving/emergency_stop`, `/full_self_driving/select_plan_artifact`, `/full_self_driving/upload_plan_artifact`).
- **Preserve Runner Workflow:** `./run_raspi.sh` remains the primary convenience wrapper for container and serial daemon management.
- **Zero Prototype Dependency:** Strictly 0 dependencies on `px4_roscon_25` legacy subpackages (`aruco_tracker`, `transit_in`, `precision_land`, etc.).
- **Zero Hardcoded Personal Paths:** No `/home/ubuntu/...` or `/home/yosh/...` fallback paths in C++ source code or launch scripts.
- **Resource Limits on Build:** All build steps must respect `--parallel-workers 2 --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=2` to ensure stability on Pi 4 and SITL hosts.

---

### Task 1: Migrate & Isolate `px4_tf` as a Clean Top-Level Package

**Files:**
- Move: `px4_roscon_25/px4_tf` -> `px4_tf`
- Modify: `px4_tf/CMakeLists.txt` (ensure clean modern ament_cmake build)
- Modify: `px4_tf/package.xml` (verify dependencies: `rclcpp`, `geometry_msgs`, `px4_msgs`, `tf2`, `tf2_ros`)
- Verify: `full_self_driving/launch/full_self_driving.launch.py:244-250`
- Verify: `full_self_driving/launch/fsd_real_flight.launch.py:140-147`

**Interfaces:**
- Consumes: `/fmu/out/vehicle_odometry` (`px4_msgs::msg::VehicleOdometry`)
- Produces: TF transforms: `odom` -> `odom_ned`, `odom_ned` -> `base_link_frd`, `base_link_frd` -> `base_link`

- [x] **Step 1: Move `px4_tf` directory from `px4_roscon_25/px4_tf` to workspace root `px4_tf`**
- [x] **Step 2: Clean up `px4_tf/CMakeLists.txt` and `px4_tf/package.xml` to ensure clean standalone build**
- [x] **Step 3: Build `px4_tf` package standalone and verify zero compile warnings**
- [x] **Step 4: Verify `px4_tf_publisher` executable is in install share**
- [x] **Step 5: Commit changes**

---

### Task 2: Purge Legacy Prototype Packages, Duplicate Files, and Dead Code

**Files:**
- Delete: `px4_roscon_25/` (remaining 8 legacy prototype packages: `aruco_database`, `aruco_database_bridge`, `aruco_tracker`, `precision_land`, `px4_roscon_25`, `search`, `transit_in`, `transit_out`)
- Delete: `gazebo_models/` (legacy worlds and obsolete `run_world.sh`)
- Delete: `full_self_driving/scripts/serial/` (vendored pyserial directory)
- Delete: `full_self_driving/config/engineering_config_simulation.yaml` (exact duplicate of `fsd_parameters.yaml`)
- Delete: `docker/docker_run_raspi.sh` (consolidated into root `./run_raspi.sh`)
- Delete: `full_self_driving/launch/fsd_companion_rpi.launch.py` (obsolete duplicate of `fsd_real_flight.launch.py`)
- Delete: `full_self_driving/launch/fsd_hitl_host.launch.py` (obsolete duplicate)
- Delete: `foxglove/extensions/fsd-mission-control/src/ExamplePanel.tsx` (unregistered boilerplate)
- Delete: `foxglove/roscon-25-workshop.json` (legacy layout; `fsd_management.json` is authoritative)

- [x] **Step 1: Delete remaining legacy folders in `px4_roscon_25/`**
- [x] **Step 2: Delete `gazebo_models/` and `full_self_driving/scripts/serial/`**
- [x] **Step 3: Delete duplicate config and duplicate shell scripts**
- [x] **Step 4: Delete obsolete launch files and unused Foxglove files**
- [x] **Step 5: Ensure workspace builds cleanly without legacy files**
- [x] **Step 6: Commit changes**

---

### Task 3: Purge Hardcoded Personal Paths (`/home/ubuntu/...` and `/home/yosh/...`)

**Files:**
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp:166-224`
- Modify: `full_self_driving/src/launch/hardware_manifest_validator.cpp:210-225`
- Modify: `full_self_driving/src/runtime/replay_fixture_publisher.cpp:65-75`
- Modify: `full_self_driving/launch/full_self_driving.launch.py:77-87, 155-180`
- Modify: `full_self_driving/launch/fsd_real_flight.launch.py:54-95`
- Modify: `full_self_driving/config/pinned_api_manifest.yaml`
- Modify: `full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml`
- Modify: Test fixtures (`search_parity_test.cpp`, `transit_in_parity_test.cpp`, `transit_out_parity_test.cpp`, `hardware_manifest_validator_test.cpp`)

- [x] **Step 1: In `flight_runtime_node.cpp`, resolve config and plan files via `ament_index_cpp::get_package_share_directory("full_self_driving")` and ROS parameters, eliminating all hardcoded `/home/yosh/` and `/home/ubuntu/` paths**
- [x] **Step 2: In `hardware_manifest_validator.cpp` and `replay_fixture_publisher.cpp`, use `get_package_share_directory` instead of hardcoded paths**
- [x] **Step 3: In `full_self_driving.launch.py` and `fsd_real_flight.launch.py`, use `FindPackageShare("full_self_driving")` for camera calibration, engineering configs, manifests, and Gazebo paths**
- [x] **Step 4: Update test files so they dynamically resolve fixture files relative to package share directory**
- [x] **Step 5: Verify all tests pass without hardcoded paths**
- [x] **Step 6: Commit changes**

---

### Task 4: Reconcile Physical Flight Configuration & Marker Size

**Files:**
- Modify: `full_self_driving/launch/fsd_real_flight.launch.py:417-420`
- Modify: `full_self_driving/config/fsd_parameters_real.yaml:41-46`
- Modify: `full_self_driving/launch/full_self_driving.launch.py:549-553`

- [x] **Step 1: Check and align `marker_size` default parameter across `fsd_real_flight.launch.py` (0.40m) and `fsd_parameters_real.yaml` (0.40m)**
- [x] **Step 2: Ensure camera calibration file auto-discovery checks `config/camera_calibrations/c270_720p.yaml` inside package share and `/root/.ros/camera_info/c270.yaml`**
- [x] **Step 3: Test launch argument overriding: verify passing `marker_size:=0.5` or `dictionary:=DICT_4X4_50` propagates properly to `fsd_perception` and `fsd_flight_runtime`**
- [x] **Step 4: Commit changes**

---

### Task 5: Enhance MicroXRCEAgent & Hardware Ergonomics in `fsd_real_flight.launch.py`

**Files:**
- Modify: `full_self_driving/launch/fsd_real_flight.launch.py`
- Modify: `run_raspi.sh`

- [x] **Step 1: In `fsd_real_flight.launch.py`, add clear console logging for `start_agent` configuration state**
- [x] **Step 2: Ensure `run_raspi.sh` helper commands (`cbuild`, `cbuild-all`) build the clean workspace smoothly**
- [x] **Step 3: Verify that Foxglove bridge (`foxglove_bridge`), camera driver, TF broadcaster, and FSD nodes launch cleanly without errors**
- [x] **Step 4: Commit changes**

---

### Task 6: Full Verification and Regression Gate

- [x] **Step 1: Clean build of entire workspace (`cbuild-all` / colcon build)**
  ```bash
  colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=2
  ```
- [x] **Step 2: Run all unit and parity tests**
  ```bash
  colcon test --packages-select full_self_driving px4_tf --parallel-workers 2
  colcon test-result --verbose
  ```
- [x] **Step 3: Perform SITL dry-run test with `full_self_driving.launch.py` (Full Sortie Mission Verified: Takeoff -> Search 6 WPs -> Precision Land -> Release Payload -> Takeoff -> Return RTL)**
- [x] **Step 4: Validate Foxglove layout compatibility with `foxglove/fsd_management.json` and create `FIELD_RUNBOOK.md`**
- [x] **Step 5: Final review and commit**

