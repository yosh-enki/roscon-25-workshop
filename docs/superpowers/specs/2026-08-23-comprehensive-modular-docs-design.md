# Design Specification: Comprehensive Modular Documentation Suite for `full_self_driving`

## 1. Context & Motivation
The `full_self_driving` codebase has evolved significantly past the original `MANUAL.md`. Recent engineering milestones include:
- Native PX4 Gripper integration (`Px4GripperPayloadAdapter` mapping `PreparePayload` to uORB `VehicleCommand`).
- ESP32 Hardware-in-the-Loop (HITL) Actuator Testbed (`esp32_gripper_actuator.ino`, Python serial bridge, DTR/RTS auto-reset draining, GPU passthrough).
- Foxglove Studio Aerospace Mission Control UI (custom `.foxe` panel extension, 3-column layout, real-time Plan Manager, target selection chips).
- Unified configuration (`config/fsd_parameters.yaml`).
- Critical flight runtime and safety bug fixes (sortie origin home lock, multi-sortie ground disarm decoupling, hard arming gates, transit-in hold resolution).

The goal of this task is to create a modular, deep-dive documentation suite inside `full_self_driving/docs/` that serves as the single source of truth for architects, flight engineers, perception engineers, hardware integrators, and operators.

---

## 2. Directory Structure & File Manifest

The documentation will reside under `full_self_driving/docs/` with the following 9 markdown documents:

```
full_self_driving/docs/
├── README.md
├── 01_architecture_and_design_principles.md
├── 02_flight_runtime_and_strategies.md
├── 03_perception_and_pad_registry.md
├── 04_payload_and_hitl_testbed.md
├── 05_foxglove_mission_control.md
├── 06_configuration_and_security.md
├── 07_development_journey_and_lessons_learned.md
└── 08_operations_and_troubleshooting_runbook.md
```

---

## 3. Module Specifications & Content Breakdown

### Module 0: `README.md` (Table of Contents & Navigation Map)
- Executive summary of the `full_self_driving` system.
- Visual navigation graph showing the relationship between documentation modules.
- Quick links and prerequisites.

### Module 1: `01_architecture_and_design_principles.md` (Architecture & Core Principles)
- Layered Architecture breakdown (Application, Domain, Flight Runtime, Perception, HAL, Persistence, Platform).
- SOLID Principles applied to robotics and ROS 2:
  - Single Responsibility: Separating perception, registry, flight strategies, and payload controllers.
  - Dependency Inversion: Flight and payload logic depending on abstract interfaces (`PayloadAdapter`, `InternalStrategy`), never hardware drivers directly.
- `px4_ros2_cpp` Registered Mode Authority:
  - Why legacy Offboard Mode (`/fmu/in/offboard_control_mode`) was rejected.
  - How `FullSelfDrivingMode` and `FullSelfDrivingModeExecutor` provide first-class PX4 mode registration, arming checks, and seamless pilot takeover.
- Single Public Launch Invariant and Hardware Deferral Gate (`HARDWARE_PROFILE_NOT_CONFIGURED`).

### Module 2: `02_flight_runtime_and_strategies.md` (Flight Runtime & Strategy Engine)
- Mission Coordinator Finite State Machine (FSM).
- Exhaustive breakdown of all 8 flight strategies:
  1. `TakeoffStrategy`: Arming prerequisites, vertical climb to cruise altitude, rate limits.
  2. `TransitInStrategy`: Route following with acceptance radius and yaw rate clamping.
  3. `DirectStrategy`: PadRegistry lookup, freshness, quality score, uncertainty gating, battery budget.
  4. `SearchStrategy`: Checkpointed boustrophedon search fallback, working plan progress sync.
  5. `PrecisionLandStrategy`: Vision-guided 5-phase landing (`HOVER_BRAKE` coast-to-stop, `APPROACH` centering at 5m AGL, `DESCEND` closed-loop velocity setpoints, `LANDED_VERIFY` dwell and ground contact check, `FINISHED`).
  6. `PayloadOperationStrategy`: Touchdown latch release, verification dwell, durable journaling.
  7. `TakeoffAfterDelivery`: Post-drop climb to egress altitude.
  8. `TransitOutStrategy` & `ReturnStrategy`: Return to Launch (RTL) to locked initial home coordinates, touchdown, and safe multi-sortie restart.
- Takeover, Failsafe, and Emergency Stop (`EmergencyStop.srv`) mechanics.

### Module 3: `03_perception_and_pad_registry.md` (Perception & Scoped Pad Registry)
- Managed Lifecycle Node (`fsd_perception`).
- `ArucoDetector` core domain:
  - Dictionary resolution (`DICT_4X4_50`, `DICT_4X4_250`, etc.).
  - OpenCV `solvePnP` 6-DoF pose estimation with corner undistortion.
  - OpenSSL SHA-256 canonical camera calibration hashing.
  - Covariance matrix estimation ($Z^2$ dependency) and Quality metric scoring ($0.0 - 1.0$).
- `TargetCoordinator`: Multi-observation qualification, spatial consistency radius, `LiveTargetLock` decisions.
- Scoped `PadRegistry`: Key indexing `(map_id, scenario_id, target_namespace, dictionary, marker_id)`, rolling observation statistics, dynamic TF2 geodetic projection from live PX4 GPS, and atomic snapshot serialization.

### Module 4: `04_payload_and_hitl_testbed.md` (Payload Subsystems & Hardware-in-the-Loop)
- Payload HAL Abstraction (`PayloadAdapter` interface).
- Concrete Adapters:
  - `SimulationPayloadAdapter`: Deterministic software mock.
  - `Px4GripperPayloadAdapter`: Mapping `PreparePayload` to PX4 uORB `VehicleCommand` (`VEHICLE_CMD_DO_GRIPPER` / `actuator_motors` / `actuator_servos`).
  - `HardwarePayloadAdapter` / ESP32 Serial Gripper Bridge.
- ESP32 Hardware-in-the-Loop Testbed:
  - Arduino firmware (`esp32_gripper_actuator.ino`) with pulse width modulation (500µs–2500µs at 50Hz).
  - Serial protocol: JSON commands (`SET_SERVO`, `PING`) and responses (`OK`, `STATUS`).
  - Python bridge (`esp32_gripper_bridge.py`) with DTR/RTS auto-reset draining.
  - Wiring diagram (ESP32 GPIO 18, SG90 servo, external 5V power, common ground).

### Module 5: `05_foxglove_mission_control.md` (Foxglove Studio Mission Control)
- Aerospace HUD 3-column layout architecture.
- Custom Extension (`roscon25.fsd-mission-control-1.0.0.foxe`):
  - Left Panel: Telemetry HUD, Pad chips (Pad 1 to Pad 6), Assign Target, Payload Open / Close & Lock controls.
  - Center Panel: 3D Scene (Drone URDF, Coordinate Frames, Landing Target, Camera frustum), Downward Video feed with annotated ArUco bounding boxes.
  - Right Panel: Plan Manager (Artifact upload, dynamic plan switching chips, real-time waypoint progress bar).
- Configurable WebSocket Bridge (`foxglove_bridge`, default port 8765).

### Module 6: `06_configuration_and_security.md` (Configuration & Security Hardening)
- Authoritative Configuration (`config/fsd_parameters.yaml`): Complete parameter catalog with data types, units, defaults, and validation rules.
- JSON Schema Enforcement (`config/schemas/`).
- SROS2 Security: Keystore generation (`generate_sros2_keystore.py`), PKI enclaves, governance, permissions XML for CycloneDDS and FastDDS.
- Gateway Security (`FsdGateway`): Command envelope validation, rate limiting (120 req/min), 8MB payload limit, negative security boundary enforcing FSM authorization.

### Module 7: `07_development_journey_and_lessons_learned.md` (Development Journey & Post-Mortems)
- Comprehensive historical analysis extracted from `git log`:
  - Journey from Task 1 Foundation to Task 16 Hardware Manifest Gate and recent HITL extensions.
  - 10+ Detailed Bug Post-Mortems (Root Cause, Impact, Fix Implementation, Architectural Lesson):
    1. Transit-in Hold & Takeover Latching Bug.
    2. Sortie Origin Home Base Coordinate Lock during Multi-Sortie.
    3. Multi-Sortie Ground Disarm vs Mid-Air Override Interlock.
    4. Auto-Takeoff Race Condition after Sortie Return.
    5. Hard Arming Rejection Gates (Target & Cargo prerequisites).
    6. ESP32 Serial DTR/RTS Auto-Reset & Bootloader Log Draining.
    7. Dynamic TF2 Geodetic Projection from Live PX4 GPS.
    8. Foxglove Port Conflict Detection & Configurable Port.
    9. Colcon Build Memory Limits and Parallel Worker OOM Protection.
    10. Payload Secured Gate in Readiness Callbacks.

### Module 8: `08_operations_and_troubleshooting_runbook.md` (Operations & Troubleshooting Runbook)
- Standard Operating Procedures (SOP):
  - Single Launch Simulation (`ros2 launch full_self_driving full_self_driving.launch.py`).
  - HITL Testbed Execution (`./scripts/run_hitl_delivery.sh` with GPU passthrough).
  - Headless CI & Acceptance Verification (`colcon test`, `test/acceptance/full_sortie_acceptance.py`).
- Operational Verification Checklist.
- Comprehensive Troubleshooting Matrix (Symptoms, Root Causes, Diagnostics, Solutions).

---

## 4. Verification & Validation Plan
1. Ensure all 9 files are created in `full_self_driving/docs/` with accurate markdown links (`file:///...`).
2. Verify all topic names, message types, service names, parameters, and source file references match the current codebase.
3. Validate that all post-mortems in Module 7 match the exact commit history and code solutions in git.
