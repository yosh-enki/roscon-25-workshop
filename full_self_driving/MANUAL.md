# Full Self-Driving (`full_self_driving`) Package Manual

This document is the **single authoritative operational and developer manual** for the `full_self_driving` ROS 2 package. All developers and AI agents working on any task must follow the guidelines, execution conventions, and single-command workflows described in this document.

---

## 0. Golden Rules & Architectural Invariants

1. **One Public Launch Entry Point**: 
   * Always use `ros2 launch full_self_driving full_self_driving.launch.py`.
   * **NEVER** use old prototype launch files or scripts (`gazebo_models/run_world.sh`, `px4_roscon_25/common.launch.py`, manual `simulation-gazebo`, or opening multiple terminal tabs manually for `MicroXRCEAgent` / `px4`).
2. **Zero Prototype Dependencies**:
   * Do **NOT** import, link, launch, or depend on prototype packages: `aruco_tracker`, `aruco_database`, `aruco_database_bridge`, `transit_in`, `transit_out`, `search`, `precision_land`, `px4_roscon_25`, or `gazebo_models`.
3. **Flight Control Exclusivity (`px4_ros2_cpp`, No Offboard)**:
   * Companion flight control uses **only** the registered `FullSelfDrivingMode` and `FullSelfDrivingModeExecutor` via `px4_ros2_cpp`.
   * Do **NOT** use `OffboardControlMode` or publish directly to `/fmu/in/offboard_control_mode` or `/fmu/in/trajectory_setpoint`.
4. **Container Execution**:
   * All builds, tests, and launches must be executed inside the Docker container (`/home/ubuntu/roscon-25-workshop_ws`).

---

## 1. Environment Sourcing

Inside the container terminal, source the environment overlays in the following order:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash
```

---

## 2. Section 1: Foundation & Integrated Simulation (Task 1)

### 2.1 The Single Simulation Launch Command

To start the complete environment (Gazebo world, PX4 SITL, MicroXRCE-DDS Agent, `/clock`, `/camera`, `/camera_info`, TF, and all FSD nodes):

```bash
# Standard launch with Gazebo GUI
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=false

# Headless launch (for CI / background testing)
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=true
```

### 2.2 What the Launch Entry Point Orchestrates Automatically

The single launch file (`launch/full_self_driving.launch.py`) manages the following graph:

```mermaid
graph TD
    LAUNCH[ros2 launch full_self_driving full_self_driving.launch.py] --> GZ[Gazebo Harmonic: kmitl_airfield.sdf]
    LAUNCH --> PX4[PX4 SITL: airframe 4014 x500_mono_cam_down]
    LAUNCH --> AGENT[MicroXRCEAgent: udp4 -p 8888]
    LAUNCH --> CLK_BR[ros_gz_bridge: /clock]
    LAUNCH --> CAM_INFO_BR[ros_gz_bridge: /camera_info]
    LAUNCH --> IMG_BR[ros_gz_image: /camera]
    LAUNCH --> RSP[robot_state_publisher: x500.urdf]
    LAUNCH --> TF[static_tf_container: map->odom, base_link->camera]
    LAUNCH --> FOX[foxglove_bridge: port 8765]
    LAUNCH --> PROBE[fsd_launch_probe: readiness monitor]
```

* **Gazebo Harmonic**: Loads the production world `simulation/worlds/kmitl_airfield.sdf` and landing pad textures.
* **PX4 SITL (`SYS_AUTOSTART=4014`)**: Starts the PX4 autopilot binary (`/home/ubuntu/px4_sitl/bin/px4`) configured for `4014_gz_x500_mono_cam_down` (x500 drone with downward-facing camera).
* **MicroXRCE-DDS Agent**: Starts `MicroXRCEAgent udp4 -p 8888`, exposing PX4 uORB topics to ROS 2 (`/fmu/in/*` and `/fmu/out/*`).
* **Clock Bridge**: Unidirectionally bridges Gazebo `/clock` to ROS 2 `/clock`.
* **Camera & Image Bridges**: Bridges `/camera_info` and `/camera` image stream from the drone's downward imager.
* **Robot State Publisher & TF**: Publishes `x500.urdf` and static coordinate frames (`map` $\rightarrow$ `odom`, `base_link` $\rightarrow$ camera optical frame).
* **Foxglove Bridge**: Exposes WebSocket server on port `8765` for 3D visualization.
* **Launch Probe (`fsd_launch_probe`)**: Verifies all simulation dependencies and prints readiness.
* **Supervised Reverse Shutdown**: Automatically cleans up all child processes on exit.

### 2.3 Hardware Deferral Gate
```bash
ros2 launch full_self_driving full_self_driving.launch.py simulation:=false
```
* Fails closed immediately with `HARDWARE_PROFILE_NOT_CONFIGURED` because physical Raspberry Pi 4 hardware bringup is explicitly deferred pending validation evidence.

### 2.4 Verification Commands (Task 1)

1. **Build the package:**
   ```bash
   cd /home/ubuntu/roscon-25-workshop_ws
   colcon build --packages-select full_self_driving --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
   ```
2. **Run the automated test suite:**
   ```bash
   source install/setup.bash
   colcon test --packages-select full_self_driving --event-handlers console_direct+
   colcon test-result --verbose
   ```
3. **Verify active topics during launch:**
   ```bash
   ros2 topic echo /clock --once
   ros2 topic echo /camera_info --once
   ros2 topic echo /fmu/out/vehicle_status_v1 --once
   ```

---

---

## 3. Section 2: Production ArUco Perception Slice (Task 2)

### 3.1 Overview & Architecture

Task 2 ports the prototype ArUco tracker into a production-grade, managed ROS 2 Lifecycle Node (`fsd_perception`). It operates on the incoming camera feed and camera intrinsics to perform marker detection, OpenCV `solvePnP` 6-DoF pose estimation, corner undistortion, covariance estimation, canonical calibration hashing, and annotated image generation.

```mermaid
graph TD
    CAM["/camera (sensor_msgs/Image)"] --> FSD_PERC[fsd_perception : LifecycleNode]
    CAM_INFO["/camera_info (sensor_msgs/CameraInfo)"] --> FSD_PERC
    
    subgraph "Core Domain (fsd_perception_core)"
        DET[cv::aruco::ArucoDetector]
        UNDIST[cv::undistortPoints]
        PNP[cv::solvePnP]
        HASH[OpenSSL SHA-256 Calibration Hash]
        COV[Covariance & Quality Metric]
    end
    
    FSD_PERC --> DET
    DET --> UNDIST --> PNP --> COV
    CAM_INFO --> HASH
    
    FSD_PERC --> OBS["/full_self_driving/perception/all_id_observations (AllIdObservationBatch)"]
    FSD_PERC --> ANNOT["/full_self_driving/perception/annotated_image (sensor_msgs/Image)"]
    FSD_PERC --> HEALTH["/full_self_driving/health (ComponentHealth)"]
```

### 3.2 Created Production Components & Artifacts

1. **Production Messages (`msg/`)**:
   * [`MessageHeader.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/MessageHeader.msg): Standard bounded header (`stamp`, `frame_id <= 64`, `sequence`).
   * [`TargetIdentity.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/TargetIdentity.msg): Marker ID and dictionary metadata (`marker_id`, `dictionary <= 32`, `target_namespace <= 64`).
   * [`AllIdObservation.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/AllIdObservation.msg): Complete 6-DoF observation record with `pose`, `covariance` (float64[36] row-major diagonal variances), `quality` (float32 [0.0, 1.0]), monotonic and camera timestamps, canonical `calibration_sha256`, and `observation_state`.
   * [`AllIdObservationBatch.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/AllIdObservationBatch.msg): Bounded array of observations (`AllIdObservation[<=256]`) with `map_id`, `scenario_id`, and drop counter.
   * [`ComponentHealth.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/ComponentHealth.msg): Node lifecycle state, ready boolean, monotonic timestamp, and diagnostics.

2. **Core Domain Library (`src/perception/`)**:
   * [`aruco_detector.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/perception/aruco_detector.hpp) / [`aruco_detector.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/perception/aruco_detector.cpp): Pure C++ domain logic without node dependencies. Encapsulates dictionary resolution (`DICT_4X4_50`, `DICT_4X4_250`, etc.), pose estimation via `solvePnP`, normalized quaternion conversion, covariance generation, and image annotation.
   * Built as target `fsd_perception_core`.

3. **Lifecycle Perception Node (`src/perception/`)**:
   * [`perception_node.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/perception/perception_node.hpp) / [`perception_node.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/perception/perception_node.cpp): `rclcpp_lifecycle::LifecycleNode` named `fsd_perception`.
   * Manages lifecycle transitions (`on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`, `on_shutdown`).
   * Fails closed if camera calibration is missing or degenerate (e.g., zero focal length).

4. **Replay Fixture Publisher & Test Fixtures (`test/fixtures/prototype_behavior/aruco/`)**:
   * [`fsd_replay_fixture_publisher`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/replay_fixture_publisher.cpp): Offline test node publishing camera stream and camera info from static fixture datasets.
   * Fixtures: `single_marker_id1.png`, `multi_marker_id1_id2.png`, `blank.png`, `camera_info.yaml`, `golden_observations.yaml`.

### 3.3 Node Parameters & Configuration

| Parameter | Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `dictionary` | string | `"DICT_4X4_50"` | ArUco dictionary name (e.g. `DICT_4X4_50`, `DICT_4X4_250`) |
| `marker_size` | double | `0.4` | Physical marker side length in meters |
| `camera_topic` | string | `"/camera"` | Input RGB/grayscale image topic |
| `camera_info_topic` | string | `"/camera_info"` | Input camera intrinsics topic |
| `camera_frame` | string | `"camera_frame"` | TF frame ID for camera optical center |
| `map_id` | string | `"kmitl_airfield"` | Allowlisted world / map identifier |
| `scenario_id` | string | `"default_scenario"` | Active scenario identifier |
| `target_namespace` | string | `"aavc2026"` | Target namespace classification |
| `autostart` | bool | `false` | When `true`, automatically configures and activates on launch |

### 3.4 Topic Interfaces & QoS

* **Subscribed Topics**:
  * `/camera` (`sensor_msgs/msg/Image`, SensorData QoS / best effort, depth=5)
  * `/camera_info` (`sensor_msgs/msg/CameraInfo`, SensorData QoS / best effort, depth=5)
* **Published Topics**:
  * `/full_self_driving/perception/all_id_observations` ([`AllIdObservationBatch`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/AllIdObservationBatch.msg), SensorData QoS, depth=10)
  * `/full_self_driving/perception/annotated_image` (`sensor_msgs/msg/Image`, SensorData QoS, depth=5)
  * `/full_self_driving/health` ([`ComponentHealth`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/ComponentHealth.msg), Transient Local / Reliable QoS, depth=10)

### 3.5 How to Run and Verify (Task 2)

#### A. Standard Live Launch (Simulation + Perception)
```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  dictionary:=DICT_4X4_50 \
  marker_size:=0.4
```

#### B. Offline Fixture Replay Mode (Deterministic Verification)
```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=true \
  replay_fixture:=aruco
```

#### C. Inspect Perception Topics
```bash
# Check perception health status
ros2 topic echo /full_self_driving/health --once

# Stream detected ArUco observations and 6-DoF poses
ros2 topic echo /full_self_driving/perception/all_id_observations

# Verify annotated image feed rate (target: 10 Hz)
ros2 topic hz /full_self_driving/perception/annotated_image
```

#### D. Manual Lifecycle Management (When `autostart:=false`)
```bash
# Check current lifecycle state
ros2 lifecycle get /fsd_perception

# Configure node
ros2 lifecycle set /fsd_perception configure

# Activate node (starts publishing observations and annotated feed)
ros2 lifecycle set /fsd_perception activate

# Deactivate node
ros2 lifecycle set /fsd_perception deactivate
```

#### E. Run Automated Tests
```bash
# Run all package unit and regression tests inside container
colcon test --packages-select full_self_driving --event-handlers console_direct+

# Run only ArUco replay and parity tests
colcon test --packages-select full_self_driving --ctest-args -R aruco_replay --event-handlers console_direct+

# Check results
colcon test-result --verbose
```

---

## 4. Section 3: Selected-Target Live-Lock Qualification & Scoped Pad Registry (Task 3)

### 4.1 Overview & Architecture

Task 3 introduces two major architectural capabilities:
1. **Live-Lock Qualification (`LiveTargetLock`)**: Consumes the all-ID observation stream, filters for the commanded target identity (marker ID, dictionary, namespace), and evaluates quality, covariance, freshness, and spatial consistency gates to transition through `ACQUIRING` $\rightarrow$ `CANDIDATE` $\rightarrow$ `QUALIFIED` $\rightarrow$ `TARGET_LOST` states.
2. **Map/Scenario-Scoped Pad Registry (`fsd_pad_registry`)**: A dedicated lifecycle node that ingests all accepted marker observations, maintains a scoped spatial database of landing pads, and isolates records strictly by `map_id`, `scenario_id`, and `target_namespace`.

```mermaid
graph TD
    CAM["/camera (sensor_msgs/Image)"] --> FSD_PERC[fsd_perception : LifecycleNode]
    CAM_INFO["/camera_info (sensor_msgs/CameraInfo)"] --> FSD_PERC
    SEL_CMD["/full_self_driving/target_selection (TargetIdentity)"] --> FSD_PERC
    
    subgraph "Perception Node (fsd_perception)"
        DET[ArucoDetector : DICT_4X4_50, 0.4m]
        COORD[TargetCoordinator : Live-Lock Qualification]
        DET -->|All-ID Batch| COORD
    end
    
    FSD_PERC -->|Annotated 2D Image| ANNOT["/full_self_driving/perception/annotated_image"]
    FSD_PERC -->|Live Lock Stream| LOCK["/full_self_driving/perception/live_target_lock (LiveTargetLock)"]
    FSD_PERC -->|All-ID Observations| OBS["/full_self_driving/perception/all_id_observations (AllIdObservationBatch)"]
    
    subgraph "Registry Node (fsd_pad_registry)"
        REG[PadRegistry : Scoped Database]
        OBS --> REG
    end
    
    REG --> REG_SNAP["/full_self_driving/pad_registry (PadRegistrySnapshot)"]
    REG --> REG_STAT["/full_self_driving/pad_registry/status (PadRegistryStatus)"]
```

### 4.2 Production Components & Messages Added

1. **Production Messages (`msg/`)**:
   * [`LiveTargetLock.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/LiveTargetLock.msg): Target qualification state (`STATE_NO_TARGET`, `STATE_ACQUIRING`, `STATE_CANDIDATE`, `STATE_QUALIFIED`, `STATE_TARGET_LOST`), selected identity, filtered 6-DoF pose, quality, covariance, and age.
   * [`PadRecord.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PadRecord.msg): Database record for an individual landing pad (`identity`, `map_id`, `scenario_id`, latitude/longitude/altitude, uncertainty, observation count, first/last seen timestamps, calibration SHA-256).
   * [`PadRegistrySnapshot.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PadRegistrySnapshot.msg): Scoped snapshot containing all registered pads (`records[]`), map/scenario metadata, and revision.
   * [`PadRegistryStatus.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PadRegistryStatus.msg): Registry lifecycle health, record count, active revision, and durability state.

2. **Core Domain & Target Coordinator (`src/domain/`, `src/perception/`)**:
   * [`target_identity.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/target_identity.hpp): Type-safe target identity abstraction with dictionary/namespace validation and canonical hashing.
   * [`live_target_lock.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/live_target_lock.hpp): Domain model and state transition logic for live lock qualification.
   * [`target_coordinator.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/perception/target_coordinator.hpp): Coordinates all-ID observation filtering, consecutive-observation gating, spatial consistency checks, and lock loss timeout transitions. Built into `fsd_perception_core`.

3. **Pad Registry Lifecycle Node (`src/registry/`)**:
   * [`pad_registry.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/registry/pad_registry.hpp): Pure C++ thread-safe scoped spatial database. Enforces scope isolation (`map_id`, `scenario_id`), covariance gating, and observation averaging. Built into `fsd_registry_core`.
   * [`pad_registry_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/registry/pad_registry_node.hpp): `rclcpp_lifecycle::LifecycleNode` named `fsd_pad_registry`. Subscribes to `/full_self_driving/perception/all_id_observations` and publishes registry snapshots and status.

4. **Test Selection Provider Fixture (`test/fixtures/`)**:
   * [`target_selection_provider.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/target_selection_provider.hpp): Node named `fsd_target_selection_provider` providing periodic or one-shot target selection commands on `/full_self_driving/target_selection`.

### 4.3 Node Parameters & Configuration

#### `fsd_perception` (Added Task 3 Parameters):
| Parameter | Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `selected_marker_id` | int | `-1` | Initial target marker ID (`-1` = none) |
| `selected_dictionary` | string | `"DICT_4X4_50"` | Initial target dictionary |
| `selected_namespace` | string | `"aavc2026"` | Initial target namespace |
| `lock_min_quality` | double | `0.1` | Minimum quality threshold to qualify a lock |
| `lock_max_pose_age_s` | double | `0.5` | Maximum observation age before lock becomes stale |
| `lock_min_consecutive_observations` | int | `2` | Number of consecutive frames needed for `STATE_QUALIFIED` |
| `lock_target_loss_timeout_s` | double | `2.0` | Timeout after which lost target transitions to `STATE_TARGET_LOST` |

#### `fsd_pad_registry`:
| Parameter | Type | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `map_id` | string | `"kmitl_airfield"` | Active map scope |
| `scenario_id` | string | `"default_scenario"` | Active scenario scope |
| `autostart` | bool | `true` | Automatically configure and activate on launch |

### 4.4 Topic Interfaces

* **Subscribed Topics**:
  * `/full_self_driving/target_selection` ([`TargetIdentity`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/TargetIdentity.msg), Reliable QoS) — Dynamic target command.
  * `/full_self_driving/perception/all_id_observations` ([`AllIdObservationBatch`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/AllIdObservationBatch.msg), SensorData QoS) — Input to `fsd_pad_registry`.
* **Published Topics**:
  * `/full_self_driving/perception/live_target_lock` ([`LiveTargetLock`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/LiveTargetLock.msg), Reliable QoS) — Live target lock stream.
  * `/full_self_driving/pad_registry` ([`PadRegistrySnapshot`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PadRegistrySnapshot.msg), Transient Local / Reliable QoS) — Active scoped pad database snapshot.
  * `/full_self_driving/pad_registry/status` ([`PadRegistryStatus`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PadRegistryStatus.msg), Reliable QoS) — Registry status and durability.

---

### 4.5 How to Run and Verify (Task 3)

#### A. Launching Simulation with Target Selection (Terminal 1)
```bash
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=false \
  test_selection:=5
```

#### B. Flying in Simulation via QGroundControl (Terminal 2)
```bash
/home/ubuntu/QGroundControl/qgroundcontrol
```
1. Arm and Takeoff to 5–10m.
2. Fly the drone over different landing pads across the KMITL Airfield:
   * **Home Pad (Takeoff)**: $(0, 0)$ $\rightarrow$ **Marker ID 5**
   * **Search Area 1**: $(90, 85)$ $\rightarrow$ **Marker ID 1**
   * **Search Area 2**: $(210, 85)$ $\rightarrow$ **Marker ID 2**
   * **Search Area 3**: $(90, 55)$ $\rightarrow$ **Marker ID 3**
   * **Search Area 4**: $(210, 55)$ $\rightarrow$ **Marker ID 4**

#### C. Inspecting Topics & Verifying Data (Terminal 3)
```bash
# 1. Verify Live Target Lock (lock_state: 2 = STATE_QUALIFIED when over ID 5)
ros2 topic echo /full_self_driving/perception/live_target_lock

# 2. Inspect Scoped Pad Registry Snapshot (shows all discovered pads and observation counts)
ros2 topic echo /full_self_driving/pad_registry --once

# 3. Test Dynamic Target Switching in Real Time (Switch commanded target to ID 1)
ros2 topic pub /full_self_driving/target_selection full_self_driving/msg/TargetIdentity \
  "{marker_id: 1, dictionary: 'DICT_4X4_50', target_namespace: 'aavc2026'}" --once
```

#### D. Visualizing in Foxglove Studio
1. Connect Foxglove Studio to `ws://localhost:8765`.
2. Add an **Image Panel** and select topic: `/full_self_driving/perception/annotated_image`.
3. Set **Image fit** to `Contain` (or double-click the image).
4. Verify 2D video is clear, green bounding boxes track markers, marker IDs are shown, and coordinate text `X: ... Y: ... Z: ...` is rendered in yellow.

#### E. Running Automated Unit & Property Tests
```bash
# Run all package unit, replay, and property tests (28 tests total)
colcon test --packages-select full_self_driving --event-handlers console_direct+

# Run Property 6 (Registry Scope Isolation)
colcon test --packages-select full_self_driving --ctest-args -R fsd_property_6_registry_isolation --event-handlers console_direct+

# Run Property 7 (All-ID / Live-Lock Separation)
colcon test --packages-select full_self_driving --ctest-args -R fsd_property_7_all_id_live_lock --event-handlers console_direct+

# Check results
colcon test-result --verbose
```

---

## 5. Section 4: Authoritative Configuration, Mission Context & Interface Contracts (Task 4)

### 5.1 Overview & Architecture

Task 4 establishes the authoritative flight policy, operational lifecycle state machine, and concrete ROS 2 contract boundary for the entire `full_self_driving` software stack:

1. **Authoritative Engineering Configuration (`EngineeringConfig`)**:
   * All flight, safety, route, simulation, adapter, and target constraint parameters are owned by a single, validated YAML configuration file (`config/engineering_config_simulation.yaml`).
   * Evaluates bounds, positive limits, and logical relationships (e.g. `search_altitude >= approach_altitude`).
   * Generates a deterministic, lowercase 64-character **SHA-256 canonical hash** using OpenSSL (`EVP_Q_digest`). Any perturbation to any field alters the hash.
   * Operator UI (Node-RED) and ROS parameters are strictly forbidden from modifying resolved engineering policy.

2. **Authoritative Mission Context (`MissionContext`)**:
   * Owns the configuration and operational lifecycle state machine:
     `STARTUP` $\rightarrow$ `STANDBY` $\rightarrow$ `CONFIGURING` $\rightarrow$ `VALIDATING` $\rightarrow$ `COMMITTED` $\rightarrow$ `READY_FOR_OWNMODE` $\rightarrow$ `LOCKED` $\rightarrow$ `COMPLETE`.
   * **Monotonic Revision Guards**: Every valid mutation advances `selection_revision` by 1. Stale or future revision inputs are rejected without state alteration.
   * **Validation & Commit Token Gate**: Validating an operator selection generates a short-lived cryptographic validation token (`tok_...`). A commit is accepted only if the provided token matches the active validation token and revision.
   * **Readiness Gatekeeper (`check_readiness`)**: Evaluates that context is committed, target is valid, config hash is active, PX4 transport is ready, durable storage is healthy, and component health checks pass.
   * **Armed-State & Runtime Immutability (`lock`)**: When a sortie is locked / armed (`is_locked() == true`), all mutation attempts (`edit_selection`, `select_target`, `select_map_scenario`, `commit`, `set_engineering_config`) are strictly rejected.

```mermaid
sequenceDiagram
    autonumber
    actor Operator as Operator (Node-RED / CLI)
    participant MC as MissionContext (Domain Core)
    participant CFG as EngineeringConfig
    participant FSD as FSD Runtime & Perception

    Note over MC,CFG: Phase 1: Authoritative Config Loading
    MC->>CFG: validate() & compute_canonical_hash()
    CFG-->>MC: is_valid: true, canonical_hash: "3b8a..."
    MC->>MC: Transition state -> STANDBY (Rev: 1)

    Note over Operator,MC: Phase 2: Disarmed Selection & Validation
    Operator->>MC: select_target(ID: 7, expected_rev: 1)
    MC->>MC: Advance Rev to 2, state -> CONFIGURING
    Operator->>MC: validate_selection(expected_rev: 2)
    MC-->>Operator: is_valid: true, token: "tok_a1b2...", state -> VALIDATING

    Note over Operator,MC: Phase 3: Authoritative Commit & Readiness
    Operator->>MC: commit(token: "tok_a1b2...", expected_rev: 2)
    MC->>MC: Transition state -> COMMITTED, committed_rev = 2
    MC->>MC: check_readiness(px4_ok, storage_ok, health_ok)
    MC-->>FSD: state -> READY_FOR_OWNMODE

    Note over Operator,FSD: Phase 4: Sortie Lock & Immutability
    Operator->>MC: lock("mission_01", "sortie_01")
    MC->>MC: Latch locked = true, state -> LOCKED
    Operator-xMC: edit_selection() -> REJECTED (Cannot edit while locked or armed)
    FSD->>FSD: LiveTargetLock qualifies ONLY committed target 7
```

### 5.2 Production Components & Interfaces Added

1. **Production Messages (`msg/`)**:
   * [`FullSelfDrivingState.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/FullSelfDrivingState.msg): Bounded state broadcast containing `config_state` (`CONFIG_STATE_*`), `flight_phase` (`FLIGHT_PHASE_*`), `armed`, `locked`, `ready_for_mode`, `active_strategy`, `mission_id`, `sortie_id`, `config_hash`, and monotonic sequence.
   * [`VehicleTelemetry.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/VehicleTelemetry.msg): Bounded vehicle flight telemetry with armed/airborne/landed flags, battery percentage/voltage/current, WGS-84 coordinates, heading, and velocities.
   * [`LandingTargetSetpoint.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/LandingTargetSetpoint.msg): Commanded landing target setpoint with target identity, pose, frame ID, quality, and live-lock qualification flag.
   * [`FlightSafetyStatus.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/FlightSafetyStatus.msg): Bounded flight safety indicators (`SAFETY_STATUS_*`), emergency stop, geofence, battery critical, and link loss status.

2. **Production Services (`srv/`)**:
   * [`FullSelfDrivingCommand.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/FullSelfDrivingCommand.srv): Typed preparation and state transition service (`CMD_CONFIGURE`, `CMD_VALIDATE`, `CMD_COMMIT`, `CMD_RESET`, `CMD_HOLD`, `CMD_RESUME`, `CMD_ABORT`) requiring request ID, target identity, and `expected_revision`.
   * [`EmergencyStop.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/EmergencyStop.srv): Emergency stop trigger requiring `reason` and `confirmation_token`.
   * [`ClearPadRegistry.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/ClearPadRegistry.srv): Scoped pad registry clear requiring `expected_registry_revision`, `confirmation: "CONFIRM"`, returning backup reference.
   * [`BackupPadRegistry.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/BackupPadRegistry.srv): Scoped pad registry backup returning managed `backup_reference` and record count.

3. **Core Domain Library (`src/domain/`)**:
   * [`engineering_config.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/engineering_config.hpp): Pure C++ configuration domain model. Handles parsing from YAML, schema validation, relationship bounds checking, and canonical OpenSSL SHA-256 hash generation.
   * [`mission_context.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/mission_context.hpp): Pure C++ mission context domain model. Manages `OperatorSelection`, `ValidationReport`, monotonic revision tracking, commit gates, readiness evaluation, and armed/locked immutability.
   * Built into target `fsd_domain_core`.

### 5.3 Engineering Configuration Parameters

The configuration file ([`config/engineering_config_simulation.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/engineering_config_simulation.yaml)) provides authoritative values:

| Parameter Category | Field | Type | Default | Bounds / Constraints |
| :--- | :--- | :--- | :--- | :--- |
| **Meta** | `schema_version` | string | `"1.0.0"` | Must match `"1.0.0"` |
| **Meta** | `deployment_id` | string | `"kmitl_airfield_sim"` | $1 \le \text{length} \le 64$ |
| **Meta** | `profile` | string | `"simulation"` | Must be `"simulation"` or `"hardware"` |
| **Safety** | `max_altitude_m` | double | `30.0` | $> 0.0\text{ m}$ |
| **Safety** | `min_battery_percentage` | double | `20.0` | $0.0 \le \text{val} \le 100.0\%$ |
| **Safety** | `target_loss_timeout_s` | double | `3.0` | $> 0.0\text{ s}$ |
| **Routes** | `transit_in_speed_m_s` | double | `5.0` | $> 0.0\text{ m/s}$ |
| **Routes** | `transit_out_speed_m_s` | double | `5.0` | $> 0.0\text{ m/s}$ |
| **Routes** | `search_altitude_m` | double | `10.0` | $\ge \text{approach\_altitude\_m}$ |
| **Routes** | `approach_altitude_m` | double | `4.0` | $> 0.0\text{ m}$ |
| **Routes** | `landing_descent_rate_m_s`| double | `0.5` | $> 0.0\text{ m/s}$ |
| **Adapters** | `px4_transport` | string | `"px4_sitl_uxrce_dds"` | Non-empty |
| **Target Constraints**| `marker_id_min / max` | uint32 | `0` / `500` | $\text{min} \le \text{max}$ |
| **Target Constraints**| `allowed_dictionaries` | string[] | `["DICT_4X4_50", ...]` | Must contain target dictionary |
| **Target Constraints**| `allowed_namespaces` | string[] | `["aavc2026"]` | Must contain target namespace |

### 5.4 How to Run and Verify (Task 4)

#### A. Run the Complete Automated Property Test Suite (71 Tests Total)
```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# Build package with test targets enabled
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# Source workspace
source install/setup.bash

# Run all 11 test suites
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose
```

#### B. Run Individual Task 4 Property-Based Tests
```bash
# 1. Property 1: Authoritative Configuration & Hash Determinism (5 tests)
ctest --test-dir build/full_self_driving -R fsd_property_1_authoritative_config --output-on-failure

# 2. Property 2: Armed-State Immutability & Mutation Lockout (4 tests)
ctest --test-dir build/full_self_driving -R fsd_property_2_armed_immutability --output-on-failure

# 3. Property 3: Disarmed Selection & Revision Isolation (5 tests)
ctest --test-dir build/full_self_driving -R fsd_property_3_scope_isolation --output-on-failure

# 4. Property 9: Complete Ownmode Readiness Gatekeeper (5 tests)
ctest --test-dir build/full_self_driving -R fsd_property_9_readiness --output-on-failure

# 5. Property 21: Concrete Bounded ROS Interface Boundary (19 tests)
ctest --test-dir build/full_self_driving -R fsd_property_21_ros_interface_boundary --output-on-failure
```

#### C. Inspect Generated ROS 2 Interfaces
```bash
ros2 interface show full_self_driving/msg/FullSelfDrivingState
ros2 interface show full_self_driving/msg/VehicleTelemetry
ros2 interface show full_self_driving/msg/LandingTargetSetpoint
ros2 interface show full_self_driving/msg/FlightSafetyStatus
ros2 interface show full_self_driving/srv/FullSelfDrivingCommand
ros2 interface show full_self_driving/srv/EmergencyStop
ros2 interface show full_self_driving/srv/ClearPadRegistry
ros2 interface show full_self_driving/srv/BackupPadRegistry
```

---

## 6. Section 5: Managed Plan Artifacts & Working-Plan State (Task 5)

### 6.1 Overview & Architecture

Task 5 delivers production plan artifact ingestion, immutable managed storage, canonical route extraction, working-plan generation, and resume/checkpoint tracking:

1. **Immutable Managed Plan Artifacts (`PlanManager`, `PlanParser`)**:
   - Ingests raw QGroundControl `.plan` JSON bytes with size and nesting depth bounds.
   - Rejects unsafe filenames and path traversal attempts (no `/`, `\`, `..`, leading dot).
   - Walks nested mission items (supporting surveys, transect complex items, and simple items) and extracts command-16 waypoints with source indexes.
   - Extracts altitude (`CameraCalc.DistanceToSurface` or waypoint altitude parameters) and cruise speed.
   - Computes deterministic **Canonical Route SHA-256** and **Artifact SHA-256**.
   - Stores accepted artifacts under managed immutable IDs (`art_<sha256_prefix>`).
   - Strictly rejects hash-changing replacements for existing artifact IDs (Requirement 2.11 / Property 4).

2. **Working-Plan Generation & Checkpoint Tracking (`WorkingPlan`, `WorkingPlanStore`)**:
   - Creates a separate working plan record with generation counters and scope (`map_id`, `scenario_id`).
   - Tracks live search progress via `SearchCheckpoint` (`next_source_index`, `completed_waypoints`, `progress_percent`, `checkpoint_position`, `sequence`).
   - Provides safe resume semantics: `route_for_search()` starts from the interrupted entry point position (if present) followed by remaining unsearched waypoints (never silently restarts at index 0).
   - Holds at the final waypoint when all waypoints are completed.
   - Disarmed and confirmation-guarded reset: `reset("CONFIRM")` increments generation, clears checkpoint position, sets `next_source_index = 0`, `progress = 0%`, while strictly preserving the source artifact hash.

```mermaid
graph TD
    UPLOAD["UploadPlanArtifact.srv (raw bytes, safe_name)"] --> PM[PlanManager]
    PM --> PP[PlanParser: JSON DOM, Cmd 16, Altitude, Hash]
    PP --> ART[ManagedPlanArtifact: immutable, art_id, sha256]
    
    CREATE["CreateOrSelectWorkingPlan.srv"] --> PM
    PM --> WP[WorkingPlan: Generation 1, 0% Progress]
    
    SEARCH_FLIGHT["Search Strategy / Flight Runtime"] -->|Update Checkpoint| WP
    WP --> CP[SearchCheckpoint: next_index, entry_point, progress]
    
    WP --> RESUME["route_for_search() -> [EntryPoint, Remaining Waypoints...]"]
    
    RESET["ResetWorkingPlan.srv (CONFIRM)"] --> WP
    WP -->|Reset Checkpoint| WP_RESET[WorkingPlan: Generation++, 0% Progress]
```

### 6.2 Created Production Components & Artifacts

1. **Production Messages & Services (`msg/`, `srv/`)**:
   - [`msg/PlanArtifactReference.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/PlanArtifactReference.msg): Managed artifact ID, safe name, SHA-256, byte length, immutable flag.
   - [`msg/SearchCheckpoint.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/SearchCheckpoint.msg): Working plan ID, generation, next source index, optional checkpoint position, completed count, total count, progress percent, reason, sequence.
   - [`msg/WorkingPlanStatus.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/WorkingPlanStatus.msg): Working plan state (`STATE_*`), IDs, hashes, generation, checkpoint, durability, update reason.
   - [`msg/ErrorReport.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/ErrorReport.msg): Bounded error diagnostics report.
   - [`srv/UploadPlanArtifact.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/UploadPlanArtifact.srv): Ingest raw plan bytes, returns artifact reference and error report.
   - [`srv/SelectPlanArtifact.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/SelectPlanArtifact.srv): Selects managed artifact into mission context.
   - [`srv/CreateOrSelectWorkingPlan.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/CreateOrSelectWorkingPlan.srv): Generates scoped working plan from artifact.
   - [`srv/ResetWorkingPlan.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/ResetWorkingPlan.srv): Confirmation-guarded working plan reset.

2. **Core Domain & Runtime Libraries (`src/domain/`, `src/runtime/`)**:
   - [`src/domain/plan_parser.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/plan_parser.hpp): Bounded JSON parsing, command-16 waypoint extraction, finite coordinate checks, canonical route hashing.
   - [`src/domain/plan_printer.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/plan_printer.hpp): Canonical QGC plan serializer with round-trip idempotency.
   - [`src/domain/working_plan.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/working_plan.hpp): Working-plan domain model, checkpoint progression, resume route generation, and reset logic.
   - [`src/runtime/plan_manager.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/plan_manager.hpp): Thread-safe runtime manager for artifacts and working plans with atomic storage.
   - [`src/runtime/working_plan_store.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/working_plan_store.hpp): Disk persistence adapter for working plans.

3. **Test Suites & Fixtures**:
   - [`test/fixtures/plans/aavc2026_mission.plan`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/plans/aavc2026_mission.plan): Production fixture copied for parity testing.
   - [`test/property/property_4_plan_immutability.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_4_plan_immutability.cpp): Property 4 test suite (Plan immutability and safe paths).
   - [`test/property/property_5_working_plan.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_5_working_plan.cpp): Property 5 test suite (Working-plan generation correctness).
   - [`test/plan/plan_round_trip_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/plan/plan_round_trip_test.cpp): Round-trip serialization idempotency test.
   - [`test/plan/working_plan_parity_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/plan/working_plan_parity_test.cpp): Algorithm parity test against prototype SearchPlanner.

### 6.3 Verification Commands (Task 5)

#### A. Run All Automated Test Suites (99 Tests Total)
```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# Source workspace
source install/setup.bash

# Run all 15 test suites
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose
```

#### B. Run Individual Task 5 Tests
```bash
# 1. Property 4: Plan Immutability & Safe Paths (5 tests)
ctest --test-dir build/full_self_driving -R fsd_property_4_plan_immutability --output-on-failure

# 2. Property 5: Working Plan Generation Correctness (4 tests)
ctest --test-dir build/full_self_driving -R fsd_property_5_working_plan --output-on-failure

# 3. Plan Parser / Printer Round-Trip Test (2 tests)
ctest --test-dir build/full_self_driving -R plan_round_trip_test --output-on-failure

# 4. Working Plan Parity Test (2 tests)
ctest --test-dir build/full_self_driving -R working_plan_parity_test --output-on-failure
```

#### C. Inspect Generated ROS 2 Interfaces
```bash
ros2 interface show full_self_driving/msg/PlanArtifactReference
ros2 interface show full_self_driving/msg/SearchCheckpoint
ros2 interface show full_self_driving/msg/WorkingPlanStatus
ros2 interface show full_self_driving/srv/UploadPlanArtifact
ros2 interface show full_self_driving/srv/SelectPlanArtifact
ros2 interface show full_self_driving/srv/CreateOrSelectWorkingPlan
ros2 interface show full_self_driving/srv/ResetWorkingPlan
```

---

---

## 7. Section 6: Durable State, Lifecycle Supervision, Recovery Gates & Gateway (Task 6)

### 7.1 Overview & Architecture

Task 6 delivers production-grade state durability, reverse lifecycle process supervision, recovery safety gates, and the typed preparation gateway:

1. **Atomic Durability Pipeline (`PersistenceManager`)**:
   - Implements a 7-stage write pipeline: `VALIDATE` $\rightarrow$ `TEMP_WRITE` (sibling `.tmp`) $\rightarrow$ `FLUSH_FSYNC` (`fsync`) $\rightarrow$ `RENAME` (atomic `rename`) $\rightarrow$ `DIRECTORY_SYNC` (`fsync` directory) $\rightarrow$ `JOURNAL` (append to `mission_journal.jsonl`) $\rightarrow$ `BACKUP`.
   - Any fault during the write pipeline retains the previous valid state and prevents durable sequence advancement.
   - Computes canonical SHA-256 checksums over snapshot data.

2. **Ordered Lifecycle Supervision (`LifecycleSupervisor`, `EvidenceNode`)**:
   - Supervised configure and activation order: `fsd_pad_registry` $\rightarrow$ `fsd_perception` $\rightarrow$ `fsd_evidence` $\rightarrow$ `fsd_gateway`.
   - Supervised reverse deactivation and shutdown order: `fsd_gateway` $\rightarrow$ `fsd_evidence` $\rightarrow$ `fsd_perception` $\rightarrow$ `fsd_pad_registry`.
   - Immediate rollback: if any node fails during activation, already-activated nodes are deactivated in reverse order and runtime readiness is withheld.

3. **Recovery Safety Gates (`RecoveryStatus`, `ResolveRecovery`)**:
   - Detects ambiguity upon restart across snapshots, journals, working plans, executor checkpoints, payload states, registry scopes, and configuration hashes.
   - Any ambiguity enters `STATE_REQUIRED` (`safe_decision_required = true`), blocking auto-arm and auto-resume.
   - Explicit `ResolveRecovery` service requires disarmed vehicle, expected recovery revision, confirmation token, and valid decision code to transition to `STATE_RESOLVED`.

4. **Typed Preparation & Inspection Gateway (`FsdGateway`, `GatewayNode`)**:
   - Strictly enforces command envelope schema `"full_self_driving.command.v1"`.
   - Allows only allowlisted preparation and inspection commands (`select_map_scenario`, `select_target_identity`, `upload_plan_artifact`, `list_plan_artifacts`, `inspect_pad_registry`, `get_status`, etc.).
   - Explicitly rejects all flight/arm/setpoint/raw-control commands with `ERROR_FORBIDDEN_COMMAND`.
   - Rejects retained MQTT commands (`ERROR_RETAINED_COMMAND_FORBIDDEN`) and stale requests (`ERROR_STALE_REQUEST`).
   - Idempotency cache ensures duplicate request IDs return cached responses without re-triggering side effects.

### 7.2 Production Components Added

1. **Production Messages & Services (`msg/`, `srv/`)**:
   - [`msg/RecoveryStatus.msg`](file:///home/yosh/roscon-25-workshop/full_self_driving/msg/RecoveryStatus.msg): Recovery state (`STATE_*`), ambiguity codes (`AMBIGUOUS_*`), decision codes (`DECISION_*`), and durability sequences.
   - [`srv/SelectMapScenario.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/SelectMapScenario.srv): Scoped map/scenario selection.
   - [`srv/SelectTargetIdentity.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/SelectTargetIdentity.srv): Target identity selection.
   - [`srv/PreparePayload.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/PreparePayload.srv): Disarmed payload preparation.
   - [`srv/ValidateMissionContext.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/ValidateMissionContext.srv): Mission context validation.
   - [`srv/CommitMissionContext.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/CommitMissionContext.srv): Mission context commit.
   - [`srv/ResolveRecovery.srv`](file:///home/yosh/roscon-25-workshop/full_self_driving/srv/ResolveRecovery.srv): Disarmed recovery resolution.

2. **JSON Schemas (`config/schemas/`)**:
   - `snapshot.schema.json`: Schema for mission snapshot records.
   - `journal.schema.json`: Schema for journal entries.
   - `backup.schema.json`: Schema for backup records.
   - `command_envelope.schema.json`: Schema for gateway command envelope.

3. **Core Libraries & Lifecycle Nodes (`src/persistence/`, `src/runtime/`, `src/gateway/`)**:
   - [`src/persistence/persistence_manager.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/persistence/persistence_manager.hpp): Persistence manager engine. Built into `fsd_persistence_core`.
   - [`src/runtime/lifecycle_supervisor.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/lifecycle_supervisor.hpp): Lifecycle supervisor. Built into `fsd_runtime_core`.
   - [`src/runtime/evidence_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/evidence_node.hpp): `fsd_evidence` lifecycle node executable.
   - [`src/gateway/fsd_gateway.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/gateway/fsd_gateway.hpp): Gateway security core. Built into `fsd_gateway_core`.
   - [`src/gateway/fsd_gateway_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/gateway/fsd_gateway_node.hpp): `fsd_gateway` lifecycle node executable.

4. **Property Test Suites (`test/property/`)**:
   - [`test/property/property_10_gateway_boundary.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_10_gateway_boundary.cpp): Property 10 test suite (`fsd_property_10_gateway_boundary`).
   - [`test/property/property_16_durable_boundary.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_16_durable_boundary.cpp): Property 16 test suite (`fsd_property_16_durable_boundary`).
   - [`test/property/property_17_recovery_safety.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_17_recovery_safety.cpp): Property 17 test suite (`fsd_property_17_recovery_safety`).
   - [`test/property/property_18_snapshot_commit.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_18_snapshot_commit.cpp): Property 18 test suite (`fsd_property_18_snapshot_commit`).

### 7.3 How to Run and Verify (Task 6)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# 1. Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 2. Source workspace
source install/setup.bash

# 3. Run all 19 test suites (138 tests total)
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose

# 4. Run individual Task 6 property test suites
ctest --test-dir build/full_self_driving -R fsd_property_10_gateway_boundary --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_16_durable_boundary --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_17_recovery_safety --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_18_snapshot_commit --output-on-failure
```

---

---

## 8. Section 7: PX4 API Probe, Single Mode Authority & Flight Runtime (Task 7)

### 8.1 Overview & Architecture

Task 7 establishes the single registered-mode authority path with PX4 Autopilot via `px4_ros2_cpp`:

1. **Pinned API Manifest & Capabilities Probe (`Px4ApiCapabilities`, `pinned_api_manifest.yaml`)**:
   - Compares compile-time traits, method signatures, and message definitions against `config/pinned_api_manifest.yaml`.
   - Explicitly checks `px4_ros2_cpp` version (0.0.1) and `px4_msgs` version (2.0.1).
   - Validates that `FullSelfDrivingMode` and `FullSelfDrivingModeExecutor` strictly follow library-managed setpoints without fallback controllers or raw topic publishers.

2. **Single Registered Mode & Executor (`FullSelfDrivingMode`, `FullSelfDrivingModeExecutor`)**:
   - Exactly **one** mode named `"Full Self-Driving"` derived from `px4_ros2::ModeBase`.
   - Exactly **one** mode executor derived from `px4_ros2::ModeExecutorBase` (`Activation::ActivateAlways`).
   - Arming check reporter evaluates comprehensive preflight readiness (lifecycle, recovery, config, transport).
   - Instant RC / QGC takeover handling: when PX4 or QGroundControl switches away from the mode, `onDeactivate` triggers immediate yield and sets takeover hold without fighting the operator.

3. **Coordinator-Owned Transitions (`MissionCoordinator`, `InternalStrategy`)**:
   - Enforces **Property 12**: all flight phase transitions are strictly owned and decided by `MissionCoordinator`.
   - Perception callbacks publish data/decisions only; they are strictly forbidden from directly commanding mode changes or setpoints.

4. **Safety Authority & Lifecycle Registration Precedence (`FlightRuntimeNode`)**:
   - Enforces **Property 20**: stronger safety authority always supersedes autonomous requests; RC/QGC takeover locks out autonomous setpoint overrides.
   - Enforces **Property 22**: all required lifecycle nodes (`fsd_pad_registry`, `fsd_perception`, `fsd_evidence`, `fsd_gateway`), storage recovery, configuration hash, and PX4 transport must be fully active and healthy *before* mode registration is permitted.

### 8.2 Production Components Added

1. **API Manifest & Adapters (`config/`, `src/adapters/`)**:
   - [`config/pinned_api_manifest.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/pinned_api_manifest.yaml): Versioned PX4 ROS 2 C++ API manifest.
   - [`src/adapters/px4_api_capabilities.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/adapters/px4_api_capabilities.hpp): Compile-time trait validation and manifest verification. Built into `fsd_adapters_core`.
   - [`src/adapters/px4_state_cache.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/adapters/px4_state_cache.hpp): Thread-safe PX4 vehicle state, odometry, home, land detection, and freshness timeouts. Built into `fsd_adapters_core`.

2. **Flight Core & Domain (`src/flight/`, `src/domain/`)**:
   - [`src/flight/internal_strategy.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/internal_strategy.hpp): Internal strategy interface for internal flight behaviors.
   - [`src/flight/full_self_driving_mode.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/full_self_driving_mode.hpp): `FullSelfDrivingMode` implementation with goto setpoints and arming check reporters. Built into `fsd_flight_core`.
   - [`src/flight/full_self_driving_mode_executor.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/full_self_driving_mode_executor.hpp): `FullSelfDrivingModeExecutor` implementation with takeover callbacks. Built into `fsd_flight_core`.
   - [`src/domain/mission_coordinator.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/mission_coordinator.hpp): Coordinator state machine for internal strategies and takeover management. Built into `fsd_flight_core`.

3. **Runtime Executable (`src/runtime/`)**:
   - [`src/runtime/flight_runtime_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/flight_runtime_node.hpp): `fsd_flight_runtime` executable orchestrating readiness evaluation, single mode registration, and telemetry publishing.

4. **Integration & Property Test Suites (`test/`)**:
   - [`test/px4_api_probe/px4_api_probe_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/px4_api_probe/px4_api_probe_test.cpp): GTest suite validating compile-time traits, method signatures, and manifest loading (`px4_api_probe_test`).
   - [`test/integration/registered_mode_authority_smoke.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/integration/registered_mode_authority_smoke.cpp): Integration test verifying registration, arming checks, takeover, and deactivation (`registered_mode_authority_smoke`).
   - [`test/property/property_12_coordinator_transitions.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_12_coordinator_transitions.cpp): Property 12 test suite (`fsd_property_12_coordinator_transitions`).
   - [`test/property/property_20_authority.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_20_authority.cpp): Property 20 test suite (`fsd_property_20_authority`).
   - [`test/property/property_22_lifecycle_registration.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_22_lifecycle_registration.cpp): Property 22 test suite (`fsd_property_22_lifecycle_registration`).

### 8.3 How to Run and Verify (Task 7)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# 1. Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 2. Source workspace
source install/setup.bash

# 3. Run all 24 test suites (163 tests total)
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --verbose

# 4. Run individual Task 7 test suites
ctest --test-dir build/full_self_driving -R px4_api_probe_test --output-on-failure
ctest --test-dir build/full_self_driving -R registered_mode_authority_smoke --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_12_coordinator_transitions --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_20_authority --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_22_lifecycle_registration --output-on-failure
```

---

## 9. Section 8: Takeoff & TransitIn Internal Flight Strategies (Task 8)

### 9.1 Overview & Architecture

Task 8 establishes the first autonomous flight strategies ported from the prototype baseline into the production `full_self_driving` architecture:

1. **Internal Flight Strategies**:
   - `TakeoffStrategy` ([`src/flight/strategies/takeoff_strategy.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/takeoff_strategy.hpp)): Manages vehicle climb from home altitude to configured search altitude (e.g. 10.0m / 15.0m), evaluates altitude arrival within configured tolerance (default: 1.0m) and vertical speed settling ($|v_z| \le 0.5\text{ m/s}$) across multiple consecutive cycles before signaling strategy completion.
   - `TransitInStrategy` ([`src/flight/strategies/transit_in_strategy.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/transit_in_strategy.hpp)): Internal strategy executing global goto setpoint waypoint navigation using `px4_ros2::GotoGlobalSetpointType`. Implements 100% behavioral parity with the proven prototype `TransitIn` algorithm including:
     - First-setpoint guard cycle ensuring map projection initialization.
     - Course heading alignment (ground speed $\ge 0.3\text{ m/s}$ computes $\text{atan2}(v_y, v_x)$ heading, with fallback to previous valid heading and local yaw).
     - Geodesic horizontal distance evaluation (`px4_ros2::horizontalDistanceToGlobalPosition`), altitude tolerance checks, and vertical speed settle gates.
     - Durable route checkpointing via `PersistenceManager` upon each waypoint arrival.

2. **Route Value Object (`domain::Route`, `domain::RoutePoint`)**:
   - Encapsulates geographic waypoints (`latitude_deg`, `longitude_deg`, `altitude_m`) and flight limits.
   - Enforces hard safety caps: maximum horizontal speed ($\le 10.0\text{ m/s}$), maximum vertical speed ($\le 3.0\text{ m/s}$), maximum heading rate ($\le 180^\circ/\text{s}$), maximum altitude ($\le 120.0\text{ m}$), and maximum data timeout ($\le 10.0\text{ s}$).
   - Supports YAML loading from both sequence-of-maps and flattened coordinate formats.

3. **Coordinator-Driven Execution Flow**:
   - `WAITING_FOR_MODE` $\rightarrow$ `TAKEOFF` $\rightarrow$ `TRANSIT_IN` $\rightarrow$ `ACQUIRE_TARGET`.
   - Strategies are attached dynamically to the single registered `FullSelfDrivingMode` without creating extra ROS 2 nodes or PX4 mode registrations.
   - Immediate takeover authority: any operator manual takeover (RC switch or QGC mode change) triggers `onDeactivate` and forces immediate transition to `HOLD`.

### 9.2 Production Components & Files Added

1. **Domain Route Objects**:
   - [`src/domain/route.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/route.hpp) / [`src/domain/route.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/route.cpp): Value objects for route definitions, waypoint validation, and safety cap clamping. Built into `fsd_domain_core`.

2. **Flight Strategies**:
   - [`src/flight/strategies/takeoff_strategy.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/takeoff_strategy.hpp) / [`src/flight/strategies/takeoff_strategy.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/takeoff_strategy.cpp): Takeoff strategy implementation. Built into `fsd_flight_core`.
   - [`src/flight/strategies/transit_in_strategy.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/transit_in_strategy.hpp) / [`src/flight/strategies/transit_in_strategy.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/transit_in_strategy.cpp): TransitIn strategy implementation. Built into `fsd_flight_core`.

3. **Fixtures & Tests**:
   - [`test/fixtures/prototype_behavior/transit_in/golden_waypoints.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior/transit_in/golden_waypoints.yaml): Baseline TransitIn waypoints fixture.
   - [`test/fixtures/prototype_behavior/transit_in/golden_transit_in_trace.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior/transit_in/golden_transit_in_trace.yaml): Golden telemetry trace fixture.
   - [`test/fixtures/prototype_behavior_map.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior_map.yaml): Updated mapping with safety change IDs (`CHG_TRANSIT_001` through `CHG_TRANSIT_004`).
   - [`test/flight/transit_in_parity_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/flight/transit_in_parity_test.cpp): 5-part parity test suite (`transit_in_parity_test`).

### 9.3 How to Run and Verify (Task 8)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# 1. Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 2. Source workspace
source install/setup.bash

# 3. Run all 25 test suites (169 tests total)
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --all --verbose

# 4. Run TransitIn parity test directly
ctest --test-dir build/full_self_driving -R transit_in_parity_test --output-on-failure
```

### 9.4 SITL Integration Issues Encountered & Solutions (Knowledge Base)

During Gazebo SITL simulation testing and bringup, three critical runtime integration challenges were diagnosed and resolved:

```mermaid
graph TD
    A[QGC Arm & Mode Select] --> B[FullSelfDrivingModeExecutor: ActivateOnlyWhenArmed]
    B --> C[Compute Relative Takeoff Alt: Home AMSL + 10m]
    C --> D[PX4 Auto-Takeoff Sequence]
    D --> E[Takeoff Complete Callback]
    E --> F[scheduleMode: FullSelfDrivingMode ID 23]
    F --> G[TransitInStrategy Waypoint Follow: 5 m/s, 4m Acceptance Rad, 45 deg/s Yaw Rate]
```

1. **PX4 Takeoff Rejection & ModeExecutor Lifecycle**:
   * **Symptom**: Calling `takeoff(...)` immediately upon mode selection resulted in `[EXECUTOR] Takeoff failed with result: Rejected` and `MAV_CMD_COMPONENT_ARM_DISARM command temporarily rejected`.
   * **Root Cause**: In PX4, `MAV_CMD_NAV_TAKEOFF` sent via `vehicle_command` is rejected if the vehicle is not armed. Using `ModeExecutorBase::Settings::Activation::ActivateAlways` caused `onActivate()` to trigger while the vehicle was still disarmed (`is_armed=0`).
   * **Fix**: Configured `FullSelfDrivingModeExecutor` with `ModeExecutorBase::Settings{ModeExecutorBase::Settings::Activation::ActivateOnlyWhenArmed}`. When the operator selects `Full Self-Driving` and arms in QGroundControl, PX4 confirms the arming state first, activates the executor, and accepts the `takeoff(...)` command. When PX4 auto-takeoff settles, the completion callback executes `scheduleMode(ownedMode().id())` to hand over control to `FullSelfDrivingMode` airborne.

2. **Absolute AMSL vs. Relative (Above Home / Ground) Takeoff Altitude**:
   * **Symptom**: The drone climbed to 7.8m above ground instead of the configured 10.0m.
   * **Root Cause**: PX4 interprets the altitude parameter in `takeoff(altitude)` as Mean Sea Level (AMSL). KMITL Airfield ground elevation in Gazebo SITL is $+2.21\text{ m AMSL}$. Passing 10.0m commanded the drone to climb to 10.0m AMSL, resulting in $10.0 - 2.21 = 7.79\text{ m}$ height above ground.
   * **Fix**: Updated `FullSelfDrivingModeExecutor::trigger_takeoff_sequence()` to query `Px4StateCache` and compute the relative altitude target:
     $$\text{Target Altitude (AMSL)} = \text{Home Altitude (AMSL)} + \text{Configured Relative Altitude (10.0 m)}$$
     This guarantees the drone climbs exactly $10.0\text{ m}$ above the takeoff point.

3. **Latched Home Position 500ms Expiration in `px4_ros2_cpp`**:
   * **Symptom**: `TransitInStrategy` logged alternating messages `Waiting for a valid PX4 home position before starting Transit In` and `Waiting for a fresh PX4 vehicle_land_detected sample before starting Transit In`, refusing to proceed to the waypoints.
   * **Root Cause**: `px4_ros2::Subscription::lastValid()` defaults to a 500ms timeout window. PX4 publishes `/fmu/out/home_position` as a static/latched state when the home point is set at boot (not as a continuous high-rate stream). Once 500ms elapsed, `home_pos_.lastValid()` returned `false`, clearing `snapshot.home_pos_valid`.
   * **Fix**: Implemented durable in-memory caching for Home Position in [`Px4StateCache`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/adapters/px4_state_cache.cpp). Once received from PX4, the coordinates are latched and `snapshot.home_pos_valid` remains `true` throughout the entire mission flight.

4. **Standard International Route Parameters**:
   * `transit_in_speed_m_s`: **`5.0` m/s** (horizontal transit speed).
   * `acceptance_radius_m`: **`4.0` m** (standard acceptance radius per PX4 `NAV_ACC_RAD` / Nav2).
   * `max_yaw_rate_deg_s`: **`45.0` deg/s** (smooth heading slew rate).

---

## 10. Section 9: Search & Checkpointed Acquisition Strategy (Task 9)

### 10.1 Overview & Architecture

Task 9 ports the prototype `SearchMode` and `SearchPlanner` algorithms into the production `full_self_driving` architecture as an internal acquisition strategy (`SearchStrategy`), fully integrated with the authoritative `WorkingPlan` and durable `SearchCheckpoint` lifecycle:

1. **Search Strategy (`SearchStrategy`)**:
   - Implemented in [`src/flight/strategies/search_strategy.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/search_strategy.hpp) as an internal strategy of `FullSelfDrivingMode`.
   - **Climb to Search Altitude**: Upon activation, evaluates the vehicle altitude against target search altitude ($AMS\text{L} = \text{Home AMSL} + \text{Configured Relative Alt}$). If below tolerance, climbs in place (preserving current XY position and heading) before advancing waypoints.
   - **Checkpointed Route Traversal**: Follows the active `CanonicalSearchRoute` from `WorkingPlan`. If resuming from a previous interruption (`has_checkpoint_position == true`), the resumed route begins with the entry point coordinate before continuing along the remaining source waypoints.
   - **Exact Checkpoint Index Arithmetic**:
     $$next\_source\_idx = starts\_with\_entry\_point\_ ? (first\_plan\_idx + (curr\_idx > 0 ? curr\_idx - 1 : 0)) : curr\_idx + 1$$
   - **Progress & Durability**:
     $$\text{Progress \%} = \left(\frac{\text{completed\_waypoints}}{\text{total\_waypoints}}\right) \times 100.0\%$$
     Updates are committed to `PlanManager` with reason `"WAYPOINT_SETTLED"` and journaled in `PersistenceManager`.
   - **Final Waypoint Hold**: Once all waypoints in the canonical route are completed, the strategy holds station above the final waypoint (`mode_finished_ = true`) and signals completion to the mode and coordinator.
   - **Safe Deactivation Checkpoint**: On strategy deactivation or manual takeover (`on_exit()`), if the vehicle has valid global position telemetry, a checkpoint is saved with `has_checkpoint_position = true` and the current vehicle coordinates, enabling seamless resumption in subsequent sorties.

2. **Integration with `FlightRuntimeNode` & `MissionCoordinator`**:
   - `FlightRuntimeNode` owns `std::shared_ptr<PlanManager>` and publishes `full_self_driving::msg::WorkingPlanStatus` at 10Hz on `/full_self_driving/working_plan/status`.
   - `MissionCoordinator` branches `TRANSIT_IN` completion $\rightarrow$ `ACQUIRE_TARGET` $\rightarrow$ `SEARCH` (fallback path when Direct navigation is unselected or unavailable).

```mermaid
stateDiagram-v2
    [*] --> TAKEOFF
    TAKEOFF --> TRANSIT_IN: Takeoff complete
    TRANSIT_IN --> ACQUIRE_TARGET: TransitIn complete
    ACQUIRE_TARGET --> SEARCH: Direct unselected / fallback
    SEARCH --> SEARCH_HOLD: All search waypoints complete (100%)
    SEARCH --> HOLD: RC / QGC Manual Takeover (Safe Checkpoint Saved)
```

### 10.2 Production Components & Files Added

1. **Strategy Implementation**:
   - [`src/flight/strategies/search_strategy.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/search_strategy.hpp): Header definition of `SearchStrategy` internal strategy.
   - [`src/flight/strategies/search_strategy.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/search_strategy.cpp): Waypoint following, climb gate, checkpoint progression, and safe deactivation.
   - Built directly into the `fsd_flight_core` library.

2. **Domain & Runtime Integration**:
   - [`src/domain/mission_coordinator.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/mission_coordinator.hpp): Added `set_plan_manager`, custom search route/plan setters, and `ACQUIRE_TARGET -> SEARCH` strategy instantiation.
   - [`src/runtime/flight_runtime_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/flight_runtime_node.hpp): `WorkingPlanStatus` publisher on `/full_self_driving/working_plan/status`, default mission plan pre-loading, and Search strategy completion logging.

3. **Fixtures & Tests**:
   - [`test/fixtures/prototype_behavior/search/golden_search_trace.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior/search/golden_search_trace.yaml): Golden search waypoints and checkpoint trace fixture.
   - [`test/fixtures/prototype_behavior_map.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior_map.yaml): Updated mapping with safety change IDs (`CHG_SEARCH_001` through `CHG_SEARCH_004`).
   - [`test/flight/search_parity_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/flight/search_parity_test.cpp): 7-part parity and replay test suite (`search_parity_test`).

### 10.3 How to Run and Verify (Task 9)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# 1. Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 2. Source workspace
source install/setup.bash

# 3. Run all 26 test suites (177 tests total)
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --all --verbose

# 4. Run Search parity test directly
ctest --test-dir build/full_self_driving -R search_parity_test --output-on-failure
```

---

## 11. Section 10: Direct Target Acquisition Strategy & Fallback (Task 10)

### 11.1 Overview & Architecture

Task 10 implements the design-approved `DirectStrategy` internal flight strategy and deterministic target acquisition fallback in `MissionCoordinator`:

1. **Direct Strategy (`DirectStrategy`)**:
   - Implemented in [`src/flight/strategies/direct_strategy.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/direct_strategy.hpp) as an internal strategy of the single registered `FullSelfDrivingMode`.
   - **Trusted Pad Record Navigation**: Uses the current trusted `PadRecord` matching the locked `(map_id, scenario_id, target_identity)` for safe navigation positioning directly above the target pad coordinates.
   - **Safe Position Settle Gate**: Reaches and settles above the target coordinate at configured search/cruise altitude ($15.0\text{ m}$ above home) with horizontal speed $\le 0.5\text{ m/s}$, vertical speed $\le 0.5\text{ m/s}$, and altitude tolerance $\le 1.0\text{ m}$ for the configured duration ($1.0\text{ s}$).
   - **Completion & Transition**: On settling (`is_settled() == true`), persists durable journal checkpoint `EVT_DIRECT_COMPLETE` and returns control to the coordinator, transitioning to `PRECISION_LAND` (specifically `PRECISION_LAND.SEARCH`).

2. **Deterministic Coordinator Gating & Fallback (`MissionCoordinator`)**:
   - Evaluates acquisition branch criteria on transition to `ACQUIRE_TARGET`:
     1. **Pad Registry Gate**: Record exists matching locked map, scenario, target namespace, dictionary, and marker ID.
     2. **Freshness Gate**: `(now - recorded_monotonic_ns) <= trusted_record_max_age_s` ($3600.0\text{ s}$).
     3. **Quality & Uncertainty Gates**: `quality >= minimum_record_quality` and `uncertainty <= max_record_uncertainty_m` ($50.0\text{ m}$).
     4. **Path & Clearance Gates**: Coordinates are finite and within valid geographic bounds.
     5. **Energy Gate**: Battery percentage $\ge \text{min\_battery\_percentage}$ ($20.0\%$).
     6. **Direct Policy Gate**: `direct_enabled == true`.
   - **Branch Selection**:
     - If all Direct gates pass $\rightarrow$ Selects `DIRECT` (`FLY-004 / EVT_ACQUISITION_DIRECT_SELECTED`).
     - If any Direct gate fails and a valid `WorkingPlan` exists $\rightarrow$ Selects `SEARCH` (`FLY-005 / EVT_ACQUISITION_SEARCH_SELECTED`).
     - If Direct gates fail and NO valid working plan exists $\rightarrow$ Fails closed to `HOLD` (`ACQUISITION_FAILED_HOLD`) with explicit error message.
   - **In-Flight Fallback**: If Direct navigation encounters a timeout ($30.0\text{ s}$) or flight failure, triggers `EVT_DIRECT_FALLBACK` $\rightarrow$ falls back to `SEARCH` (when plan valid) or `HOLD`.

```mermaid
graph TD
    ACQ[ACQUIRE_TARGET] --> GATES{Evaluate Direct Gates:<br/>Scope, Age, Quality,<br/>Uncertainty, Energy, Path}
    GATES -->|All Pass| DIR[DIRECT Strategy<br/>FLY-004 / EVT_ACQUISITION_DIRECT_SELECTED]
    GATES -->|Gate Fails| PLAN_CHECK{Valid WorkingPlan<br/>Available?}
    PLAN_CHECK -->|Yes| SRCH[SEARCH Strategy<br/>FLY-005 / EVT_ACQUISITION_SEARCH_SELECTED]
    PLAN_CHECK -->|No| FAIL[HOLD Fail-Closed<br/>ACQUISITION_FAILED_HOLD]
    
    DIR --> ARRIVE{Arrived & Settled<br/>Above Pad?}
    ARRIVE -->|Yes| PLAND[PRECISION_LAND.SEARCH<br/>FLY-006 / EVT_DIRECT_COMPLETE]
    ARRIVE -->|Timeout / Fault| FALLBACK{Valid WorkingPlan?}
    FALLBACK -->|Yes| SRCH_FB[SEARCH Strategy<br/>FLY-007 / EVT_DIRECT_FALLBACK]
    FALLBACK -->|No| HOLD_FB[HOLD Fail-Closed]
```

### 11.2 Property 8 Invariant: Direct Never Substitutes for Live Lock

Under **Design Property 8 (Validates Requirement 3.3)**:
- Direct navigation assistance navigates the aircraft to a safe holding position above the trusted pad record, but **NEVER** creates a visual target lock, **NEVER** verifies the landing target, **NEVER** enters descent to the ground, and **NEVER** authorizes payload operations.
- The `LiveTargetLock` topic and status remain strictly unqualified (`is_qualified() == false`) throughout Direct navigation and upon Direct completion until fresh, qualified visual observations are processed.

### 11.3 Production Components & Files Added

1. **Strategy Implementation**:
   - [`src/flight/strategies/direct_strategy.hpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/direct_strategy.hpp): Header definition of `DirectStrategy` internal strategy.
   - [`src/flight/strategies/direct_strategy.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/flight/strategies/direct_strategy.cpp): Global goto navigation, geodesic arrival distance, velocity settle gates, timeout, and safe completion.
   - Built directly into `fsd_flight_core`.

2. **Domain Coordinator & Runtime**:
   - [`src/domain/mission_coordinator.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/domain/mission_coordinator.hpp): Complete gate evaluation (`is_direct_eligible`), search validation (`is_search_plan_valid`), acquisition branching (`EVT_ACQUISITION_DIRECT_SELECTED`, `EVT_ACQUISITION_SEARCH_SELECTED`), Direct completion (`EVT_DIRECT_COMPLETE`), and fallback (`EVT_DIRECT_FALLBACK`).
   - [`src/runtime/flight_runtime_node.hpp/.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/src/runtime/flight_runtime_node.hpp): Wired `PadRegistry`, added `acquisition_fixture` parameter handling (`direct`, `stale_direct`, `cross_scope_direct`, `unsafe_direct`, `search_fallback`, `no_plan_hold`), and wired Direct completion to `PRECISION_LAND`.
   - [`launch/full_self_driving.launch.py`](file:///home/yosh/roscon-25-workshop/full_self_driving/launch/full_self_driving.launch.py): Added `acquisition_fixture` launch argument and forwarded to `fsd_flight_runtime`.

3. **Property & Integration Tests**:
   - [`test/property/property_8_direct_lock_separation.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/property/property_8_direct_lock_separation.cpp): 6-part test suite verifying Property 8 (`fsd_property_8_direct_lock_separation`).
   - [`test/flight/acquisition_branch_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/flight/acquisition_branch_test.cpp): 11-part integration test suite verifying deterministic acquisition branching across all criteria (`acquisition_branch_test`).
   - [`test/fixtures/prototype_behavior_map.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/fixtures/prototype_behavior_map.yaml): Updated mapping with safety change IDs (`CHG_DIR_001`, `CHG_DIR_002`).

### 11.4 How to Run and Verify (Task 10)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash

# 1. Build package
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

# 2. Source workspace
source install/setup.bash

# 3. Run all test suites
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --all --verbose

# 4. Run Property 8 test directly
ctest --test-dir build/full_self_driving -R fsd_property_8_direct_lock_separation --output-on-failure

# 5. Run Acquisition Branch test directly
ctest --test-dir build/full_self_driving -R acquisition_branch_test --output-on-failure
```

---

## 12. Section 11: Precision Land Strategy & Vision-Guided Touchdown (Task 11)

### 12.1 Overview & State Machine Architecture

The **Precision Land Strategy** (`flight::PrecisionLandStrategy`) delivers visual precision descent and touchdown directly onto the detected ArUco landing pad while maintaining strict prototype parity, smooth momentum neutralization, and high-altitude (15.0 m) detection stability.

```mermaid
stateDiagram-v2
    [*] --> SEARCH: Enter at Cruise/Search Altitude (15.0m AGL)
    SEARCH --> HOVER_BRAKE: Target Acquired (LiveTargetLock Qualified)
    HOVER_BRAKE --> APPROACH: Forward Momentum Decelerated to 0 m/s (Dwell 1.0s)
    APPROACH --> DESCEND: Position Centered over Pad at 5.0m AGL (delta_pos < 0.25m, vel < 0.25m/s)
    DESCEND --> LANDED_VERIFY: Touchdown (is_landed == true) via 3D Velocity Vector Control (vz = 1.0 m/s)
    LANDED_VERIFY --> FINISHED: Stability Dwell (0.5s) on Ground Contact Complete
    FINISHED --> [*]
```

### 12.2 Key Architectural Features & Innovations

1. **Zero-Velocity Coasting Brake (`HOVER_BRAKE`)**:
   * When the target pad is visually acquired at high forward speed ($5.0\text{ m/s}$), the controller actively neutralizes forward momentum without commanding reverse flight.
   * Dynamically tracks instantaneous position until horizontal and vertical velocity drop below $\Delta v = 0.25\text{ m/s}$ for a $1.0\text{ s}$ stabilization dwell, then locks position and triggers `APPROACH`.

2. **Trajectory Velocity Vector Guidance (`TrajectorySetpointType`)**:
   * Operates directly with PX4's internal velocity trajectory tracking via `px4_ros2::TrajectorySetpointType`.
   * Computes signed lateral velocity setpoint $(v_x, v_y)$ via prototype proportional gain ($K_p = 1.5$, max velocity $3.0\text{ m/s}$) with discrete time integral scaling ($\Delta\text{pos} \times dt\_s$).
   * Descends steadily at constant vertical rate ($v_z = 1.0\text{ m/s}$) with yaw heading tracking aligned to the landing pad orientation.

3. **Full 3D Odometry Attitude Compensation**:
   * Uses 3D attitude quaternions from `px4_ros2::OdometryAttitude` to de-rotate camera optical rays into World NED coordinates.
   * Completely eliminates attitude-position positive feedback oscillation loops caused by drone pitching and rolling during flight.

4. **Raspberry Pi 4 & 720p / 15m Perception Tuning**:
   * **`minMarkerPerimeterRate = 0.01`**: Detects small/distant markers at 15–20m altitude without dropping frames.
   * **`cornerRefinementMethod = CORNER_REFINE_CONTOUR`**: Fast, lightweight sub-pixel corner refinement designed for ARM CPU efficiency.
   * **`adaptiveThreshWinSizeStep = 5`**: Optimized adaptive threshold search window for Raspberry Pi 4 CPU budget.
   * **Grayscale Pre-conversion**: Single-channel grayscale conversion avoids redundant BGR checks.
   * **Target EMA Low-Pass Filter ($\alpha = 0.75$)**: Smooths marker coordinates against high-altitude pixel discretization jitter.

### 12.3 Prototype Parity Parameter Reference

| Parameter | Authoritative Value | Description |
|---|---|---|
| `max_velocity` | `3.0 m/s` | Maximum lateral correction velocity limit |
| `descent_vel` | `1.0 m/s` | Constant vertical descent speed |
| `vel_p_gain` | `1.5` | Proportional gain for lateral velocity correction |
| `vel_i_gain` | `0.0` | Integral gain |
| `target_timeout` | `3.0 s` | Visual target loss threshold before fail-closed trigger |
| `delta_position` | `0.25 m` | Position convergence tolerance for `APPROACH` $\rightarrow$ `DESCEND` |
| `delta_velocity` | `0.25 m/s` | Velocity convergence tolerance for `APPROACH` $\rightarrow$ `DESCEND` |
| `search_altitude_m` | `15.0 m` | Search, cruise, and direct navigation altitude AGL |
| `approach_altitude_m`| `5.0 m` | Approach and centering altitude AGL |
| `stabilize_duration_s`| `1.0 s` | Hover brake dwell time |

### 12.4 How to Run and Verify (Task 11)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash

# 1. Build and test all suites (205 tests)
colcon build --packages-select full_self_driving --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --all --verbose

# 2. Run Precision Land Parity Test directly
ctest --test-dir build/full_self_driving -R precision_land_parity_test --output-on-failure

# 3. Launch Simulation with Direct Navigation to Pad ID 2 at 15m
ros2 launch full_self_driving full_self_driving.launch.py \
  simulation:=true \
  world:=kmitl_airfield \
  headless:=false \
  test_selection:=2 \
  acquisition_fixture:=trusted_direct
```

---

## 13. Section 12: Payload Delivery & Sortie Completion (Task 12)

### 13.1 Overview & Post-Touchdown Pipeline

Task 12 implements the post-touchdown mission sequence that completes the delivery sortie:

```mermaid
stateDiagram-v2
    [*] --> WAITING_FOR_MODE
    WAITING_FOR_MODE --> TAKEOFF : Armed & Mode Registered
    TAKEOFF --> TRANSIT_IN : Takeoff Complete (10m AGL)
    TRANSIT_IN --> DIRECT : Target in Registry (Direct Eligible)
    TRANSIT_IN --> SEARCH : Fallback Search Plan
    DIRECT --> PRECISION_LAND : Approach Waypoint Reached
    SEARCH --> PRECISION_LAND : Target Locked & Qualified
    PRECISION_LAND --> LANDED_VERIFIED : Touchdown & Stability Dwell (0.5s)
    LANDED_VERIFIED --> PAYLOAD_OPERATION : Gates Evaluated & Passed
    PAYLOAD_OPERATION --> TAKEOFF_AFTER_DELIVERY : EVT_PAYLOAD_SUCCESS (Result=1)
    PAYLOAD_OPERATION --> RETURN_STRATEGY : EVT_PAYLOAD_UNKNOWN / FAILURE
    TAKEOFF_AFTER_DELIVERY --> TRANSIT_OUT : Climb Complete (15m AGL)
    TRANSIT_OUT --> RETURN_STRATEGY : Outbound Waypoints Settled
    RETURN_STRATEGY --> RETURN_LANDED : Touchdown at Home Pad & Disarm
    RETURN_LANDED --> [*] : EVT_SORTIE_COMPLETED
```

### 13.2 Architecture & Safety Invariants

#### A. Multi-Layer Safety Model for Payload Actuation
1. **Preflight Hardware Preparation (`PreparePayload`)**:
   - Pilot / Operator can open winch/servo for loading (`OP_OPEN_FOR_LOADING`), verify lock (`OP_VERIFY_SECURED`), or perform final preflight check (`OP_PREPARE_FOR_SORTIE`).
   - Preflight operations are permitted **ONLY when the vehicle is DISARMED and UNLOCKED**.
   - FSD Preflight Readiness Gate strictly requires `FEEDBACK_SECURED` before arming or mode registration is allowed.
2. **In-Flight Release Interlock Barrier**:
   - In-flight release commands via external APIs (Gateway, Node-RED, Web UI, MQTT) are **strictly rejected**.
   - Physical payload actuation is commanded **ONLY internally** by `PayloadOperationStrategy` after touchdown verification (`LANDED_VERIFIED`).
3. **Durable Intent & Idempotency**:
   - Before commanding the physical actuator, `PayloadOperationStrategy` logs a durable intent record `EVT_PAYLOAD_INTENT` with an idempotent operation key into the journal.
   - Duplicate release requests with the same `operation_id` return the cached result immediately without re-actuating hardware or double-incrementing counters.
4. **No Auto-Retry on Fault / Unknown**:
   - In the event of actuator timeout, power loss, or contradictory feedback, the system records `RESULT_UNKNOWN` (`EVT_PAYLOAD_UNKNOWN`).
   - The mission coordinator **NEVER automatically retries** an unknown release in-flight or on the pad; it safely aborts to `RETURN_STRATEGY`.

#### B. Explicit Return Corridor Deconfliction (Property 15)
- The outbound transit corridor (`TRANSIT_OUT`) and return strategy mode (`RETURN_TO_HOME`, `LAND_IMMEDIATELY`, `HOLD_AT_FINAL_WAYPOINT`) are explicitly defined in the authoritative mission configuration and snapshots.
- The system **never assumes automatic inbound route reversal** in memory, guaranteeing dedicated altitude separation (15m AGL transit out vs. 10m AGL transit in) and collision avoidance along congested airfield corridors.

### 13.3 Production Components Added

| Component | Path | Responsibility |
|---|---|---|
| `PayloadAdapter` | `full_self_driving/src/payload/payload_adapter.hpp` | Virtual HAL interface for payload release hardware (servos, winches, electro-magnets) |
| `SimulationPayloadAdapter` | `full_self_driving/src/payload/simulation_payload_adapter.hpp` | Deterministic simulation mock with fault injection modes (`FAULT_TIMEOUT`, `FAULT_CONTRADICTORY_FEEDBACK`, `FAULT_HARDWARE_ERROR`, `FAULT_POWER_LOSS`) |
| `PayloadController` | `full_self_driving/src/payload/payload_controller.hpp` | Domain controller managing preflight operations, internal release, idempotency records, and readiness queries |
| `PayloadOperationStrategy` | `full_self_driving/src/flight/strategies/payload_operation_strategy.hpp` | Internal strategy executing pre-drop safety gate checks, durable intent journaling, actuation, and result verification |
| `TransitOutStrategy` | `full_self_driving/src/flight/strategies/transit_out_strategy.hpp` | Outbound waypoint navigation strategy with course heading alignment, velocity settling gates, and durable progress journaling |
| `ReturnStrategy` | `full_self_driving/src/flight/strategies/return_strategy.hpp` | Configurable return strategy managing home approach, vertical touchdown descent, dwell verification, and sortie completion |

### 13.4 ROS Interfaces Added

- **Service**: `/full_self_driving/prepare_payload` (`full_self_driving/srv/PreparePayload`)
  - Request: `uint8 operation`, `string request_id`, `uint64 expected_selection_revision`
  - Response: `bool accepted`, `PayloadStatus status`, `bool has_error`, `ErrorReport error`
- **Topic**: `/full_self_driving/payload/status` (`full_self_driving/msg/PayloadStatus`)
  - Latched publisher (1 Hz) streaming current feedback state, cargo status, operation counts, and hardware health.

### 13.5 How to Run and Verify (Task 12)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash

# 1. Run all 240 package tests (100% pass)
colcon test --packages-select full_self_driving --event-handlers console_direct+
colcon test-result --all --verbose

# 2. Run Task 12 specific test suites
ctest --test-dir build/full_self_driving -R payload_controller_test --output-on-failure
ctest --test-dir build/full_self_driving -R payload_operation_test --output-on-failure
ctest --test-dir build/full_self_driving -R transit_out_parity_test --output-on-failure
ctest --test-dir build/full_self_driving -R return_strategy_test --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_14_payload_safety --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_15_return_strategy_explicitness --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_property_11_mission_sequence --output-on-failure
```

---

## 14. Section 13: Security & Multi-Machine DDS Hardening (Task 13)

### 14.1 Overview & Defense-in-Depth Model

Task 13 introduces complete cryptographic authentication, granular SROS2 access controls, transport-level payload encryption, and network segmentation to harden the Full Self-Driving stack against rogue network nodes, eavesdropping, and unauthorized flight command injection.

```
[ FLIGHT-CRITICAL ONBOARD DOMAIN ]                [ AIRLOCK GATEWAY ]               [ GROUND / TELEMETRY PLANE ]
Domain ID: 0 (Strict SROS2 Enforced)                                                TLS / MQTT / External Network
+------------------------------------+             +-------------------+             +--------------------------+
|  fsd_flight_runtime                |             |                   |             |  QGroundControl          |
|  fsd_perception                    | <---------> |    fsd_gateway    | <---------> |  Node-RED Dashboard      |
|  fsd_pad_registry                  |             |                   |             |  Web Telemetry UI        |
|  fsd_evidence                      |             +-------------------+             +--------------------------+
|  PX4 MicroXRCE-DDS Agent           |             (Strict Command      
+------------------------------------+              Filter & Bounded
(AES-GCM-256 Encrypted SHM/Unicast)                 Payload Interlock)
```

### 14.2 SROS2 PKI & Automated Keystore Management

Every autonomy node runs inside an isolated security enclave verified by an X.509 digital certificate signed by the Authoritative Root CA:

```
sros2_keystore/
├── public/
│   ├── ca.cert.pem                   # FSD Root CA Certificate
│   └── identity_ca.cert.pem          # CA public anchor for node identity
├── private/
│   └── ca.key.pem                    # Root CA Private Key (0600 root-only)
└── enclaves/
    └── full_self_driving/
        ├── flight_runtime/           # Enclave for /full_self_driving/fsd_flight_runtime
        │   ├── cert.pem, key.pem, identity_ca.cert.pem, permissions_ca.cert.pem
        │   ├── governance.xml, governance.p7s
        │   └── permissions.xml, permissions.p7s
        ├── perception/               # Enclave for /full_self_driving/fsd_perception
        ├── pad_registry/             # Enclave for /full_self_driving/fsd_pad_registry
        ├── gateway/                  # Enclave for /full_self_driving/fsd_gateway
        └── evidence/                 # Enclave for /full_self_driving/fsd_evidence
```

**Management Tooling**:
- `full_self_driving/scripts/generate_sros2_keystore.py`: Automated generator provisioning Root CA, node keys, X.509 certificates with Subject DN (`CN=/full_self_driving/<enclave>`), OMG DDS Security `governance.xml`, and PKCS#7 (`.p7s`) signed policies.
- `full_self_driving/scripts/manage_sros2_keystore.sh`: Operations CLI supporting `generate`, `verify`, `inspect`, `rotate`, and `clean`.

### 14.3 Granular Access Control Matrix (`permissions.xml`)

Each node enclave enforces strict **least-privilege** allowlists with default `DENY`:

| Node Enclave | Allowed Subscriptions | Allowed Publications | Allowed Services / Clients |
|---|---|---|---|
| `/full_self_driving/flight_runtime` | `/full_self_driving/live_target_lock`, `/fmu/out/*`, `/clock`, `/tf`, `/tf_static` | `/full_self_driving/state`, `/full_self_driving/readiness`, `/full_self_driving/safety`, `/full_self_driving/telemetry`, `/full_self_driving/plan/working_status`, `/full_self_driving/payload/status` | Server: `/full_self_driving/emergency_stop`, `/full_self_driving/prepare_payload` |
| `/full_self_driving/perception` | `/camera`, `/camera_info`, `/full_self_driving/target_selection`, `/clock`, `/tf`, `/tf_static` | `/full_self_driving/all_id_observations`, `/full_self_driving/annotated_image`, `/full_self_driving/live_target_lock`, `/full_self_driving/health/perception` | (None) |
| `/full_self_driving/pad_registry` | `/full_self_driving/all_id_observations`, `/clock` | `/full_self_driving/pad_registry/snapshot`, `/full_self_driving/pad_registry/status`, `/full_self_driving/health/pad_registry` | (None) |
| `/full_self_driving/evidence` | `/full_self_driving/state`, `/full_self_driving/payload/status`, `/full_self_driving/safety`, `/clock` | `/full_self_driving/health/evidence` | (None) |
| `/full_self_driving/gateway` | `/full_self_driving/state`, `/full_self_driving/telemetry`, `/full_self_driving/readiness`, `/full_self_driving/payload/status`, `/full_self_driving/plan/working_status`, `/full_self_driving/pad_registry/snapshot`, `/clock` | `/full_self_driving/health/gateway`, `/full_self_driving/target_selection` | Client: `/full_self_driving/prepare_payload`, `/full_self_driving/emergency_stop` |

### 14.4 DDS Security & Multi-Machine Transport Profiles

1. **FastDDS Profile (`config/security/fastdds_security.xml`)**:
   - Authentication Plugin: `builtin.PKI-DH`
   - Access Control Plugin: `builtin.Access-Permissions`
   - Cryptographic Plugin: `builtin.AES-GCM-256`
   - Transport: Intra-host Shared Memory (SHM) + UDPv4
2. **CycloneDDS Profile (`config/security/cyclonedds_security.xml`)**:
   - Disabled unauthenticated multicast on external interfaces (`<AllowMulticast>false</AllowMulticast>`).
   - Unicast discovery peer lists (`<Peer address="127.0.0.1"/>`).
   - Cryptographic plugin with AES-GCM-256.

### 14.5 Safety Property 26: Security Rejection Has No Flight Side Effect

- **Validates**: Requirements 4.2, 7.6, 7.7
- Any unauthorized DDS participant, rogue command injection (`arm`, `takeoff`, `raw_actuator`, `override`), stale request age, retained MQTT command, or malformed schema is rejected immediately.
- Rejection produces **zero** state changes in the `MissionCoordinator`, **zero** physical release actuations, and fails closed with durable error reporting.

### 14.6 How to Run and Verify (Task 13)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash

# 1. Manage and verify SROS2 Keystore
./src/roscon-25-workshop/full_self_driving/scripts/manage_sros2_keystore.sh verify
./src/roscon-25-workshop/full_self_driving/scripts/manage_sros2_keystore.sh inspect

# 2. Run Task 13 specific security test suites
ctest --test-dir build/full_self_driving -R fsd_property_26_security_rejection --output-on-failure
pytest-3 src/roscon-25-workshop/full_self_driving/test/security/security_policy_enforcement_test.py -v
pytest-3 src/roscon-25-workshop/full_self_driving/test/security/forbidden_dependency_scan.py -v

# 3. Run all 273 package tests (100% pass)
colcon test --packages-select full_self_driving
colcon test-result --all --verbose
```

---

## 15. Section 14: Negative Security Boundaries, Resource Limits & Observability Safety (Task 14)

### 15.1 Overview & Negative Safety Model

Task 14 establishes comprehensive negative security boundary proofs, resource exhaustion protections, adapter failure resilience, and observability noninterference across the entire repository.

```
+---------------------------------------------------------------------------------------------------+
|                                 FSD AUTONOMY SAFETY SHIELD (TASK 14)                                |
+---------------------------------------------------------------------------------------------------+
|  1. Property 25: Observability Noninterference & Truthfulness                                     |
|     - Bounded async telemetry / logging; exporter stalls cannot jitter or seize flight loop.      |
|     - Zero QGC presence inference; stale projections cannot claim authority.                     |
|                                                                                                   |
|  2. Repository-Wide Production Boundary Scan (production_boundary_scan.py)                        |
|     - Zero prototype imports/links/launches (`px4_roscon_25`, `transit_in`, `aruco_tracker`).     |
|     - Zero raw Offboard topics (`/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`).   |
|     - Strict ROS message bounding (`string<=N`, `sequence[<=N]`).                                |
|                                                                                                   |
|  3. Gateway Negative Security & Fuzzing (fsd_gateway_security_test)                               |
|     - Clock skew & future timestamp drift rejection (`ERROR_CLOCK_SKEW`).                         |
|     - Retained command rejection (`ERROR_RETAINED_COMMAND_FORBIDDEN`).                            |
|     - Malformed JSON, oversized payloads (>1MiB), shell & path injection fuzzing.                 |
|                                                                                                   |
|  4. Resource Bounds & Adapter Failure Proof (fsd_resource_failure_test)                           |
|     - Failing payload hardware adapter fails closed (`RESULT_HARDWARE_ERROR`).                    |
|     - In-flight payload delivery timeouts record `RESULT_UNKNOWN` and safe recovery climb.        |
|     - Observation / evidence queue drop backpressure without memory leaks or flight stalls.      |
+---------------------------------------------------------------------------------------------------+
```

### 15.2 Safety Property 25: Observability Noninterference and Truthfulness

- **Validates**: Requirements 7.1, 7.5.
- Logging, diagnostics, metrics, and traces operate via bounded asynchronous buffers.
- Even under maximum persistence journal backlog, storage reserve warnings, or exporter stalls, the real-time flight mode execution loop completes in bounded time (< 50ms).
- Observability and telemetry components cannot select modes, alter flight phases, disarm the vehicle, or publish raw actuator commands.
- Telemetry never infers QGroundControl GUI presence from network traffic alone; authority shifts only upon verified PX4 failsafe or RC trigger.

### 15.3 Repository-Wide Production Boundary Scan

- Automated AST and static analysis scanner ([`test/security/production_boundary_scan.py`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/security/production_boundary_scan.py)):
  1. Scans all source, headers, launch files, schemas, and configurations.
  2. Verifies zero occurrence of prototype package dependencies (`px4_roscon_25`, `aruco_tracker`, `transit_in`, `precision_land`, etc.).
  3. Verifies zero raw Offboard symbols (`OffboardControlMode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/offboard_control_mode`).
  4. Enforces bounded fields across all 20 `.msg` and 10 `.srv` definition files.
  5. Includes negative self-test validating that injected forbidden patterns fail the scan.

### 15.4 Gateway Negative Security & Fuzzing Test Suite

- Integration test ([`test/security/gateway_security_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/security/gateway_security_test.cpp)):
  1. **Clock Skew**: Rejects commands with timestamps skewed into the future (`ERROR_CLOCK_SKEW`) or stale in the past (`ERROR_STALE_REQUEST`).
  2. **Retained Messages**: Rejects MQTT retained messages with `ERROR_RETAINED_COMMAND_FORBIDDEN`.
  3. **Fuzzing & Malformed Payloads**: Tests non-JSON garbage, truncated JSON, oversized payloads (`ERROR_PAYLOAD_TOO_LARGE`), and deep recursion.
  4. **Command & Path Injection**: Proves shell injections (`$(rm -rf /)`), path traversals (`../../../../etc/passwd`), and SQL injections produce zero physical actuation or state mutation.
  5. **Zero Side Effect Invariant**: Vehicle remains disarmed, coordinator strategy is unchanged, and payload operation counts do not increment.

### 15.5 Resource Bounds & Adapter Failure Proof

- Integration test ([`test/integration/resource_failure_test.cpp`](file:///home/yosh/roscon-25-workshop/full_self_driving/test/integration/resource_failure_test.cpp)):
  1. **Hardware Error Fault Mode**: Adapter hardware error during sortie preparation returns `accepted=false`, sets `RESULT_HARDWARE_ERROR`, and prevents arming.
  2. **Timeout Fault Mode**: In-flight release timeout records explicit `RESULT_UNKNOWN` and triggers safe recovery transition.
  3. **Unhealthy / Power Loss Adapter**: Blocks mission preparation and fails closed.
  4. **Emergency Stop Interlock**: Immediately forces coordinator into `EMERGENCY_STOP` strategy and blocks further flight transitions.
  5. **Queue Backpressure**: High-frequency pad observation ingestion handles queue drops gracefully without memory growth or assertion failures.

### 15.6 How to Run and Verify (Task 14)

```bash
cd /home/ubuntu/roscon-25-workshop_ws
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros_ws/install/setup.bash
source install/setup.bash

# 1. Run Task 14 specific test targets
ctest --test-dir build/full_self_driving -R fsd_property_25_observability_noninterference --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_gateway_security_test --output-on-failure
ctest --test-dir build/full_self_driving -R fsd_resource_failure_test --output-on-failure
pytest-3 src/roscon-25-workshop/full_self_driving/test/security/production_boundary_scan.py -v

# 2. Run complete test suite across all units, properties, and integrations
colcon test --packages-select full_self_driving
colcon test-result --all --verbose
```

---

## 16. Subsequent Task Sections (To Be Extended by Other Tasks)

* **Section 15: End-to-End Mission Rehearsal & Live Mission Acceptance (Task 15)** — Full mission soak testing, multi-sortie cycle validation, and final acceptance verification.




