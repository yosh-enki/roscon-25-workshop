# Design Document: Task 15 — Complete Exact Simulation Acceptance & Adapter-Invariance Proof

**Author:** Antigravity (Google DeepMind Pair Programming Assistant)  
**Date:** 2026-08-18  
**Status:** PROPOSED (Ready for User Review)  
**Checkpoint:** Checkpoint G (Final Acceptance, Regression & Adapter Invariance)  

---

## 1. Executive Summary

Task 15 represents the final acceptance milestone of the Full Self-Driving (FSD) autonomy stack for ROSCon 2025. With all previous vertical slices (Tasks 1 through 14) fully implemented and verified with 100% test pass across 295 unit, property, security, and integration tests, Task 15 establishes:

1. **Task 15.1: End-to-End Simulation Acceptance Suite (`test/acceptance/full_sortie_acceptance.py`)**:
   A deterministic, high-fidelity end-to-end acceptance suite testing the complete 14-stage autonomous sortie lifecycle and 5 critical safety fault injection branches without relying on brittle real-time sleep loops.
2. **Task 15.2: Clean Install, One-Launch Ownership & Isolated Workspace Regression**:
   Rigorous verification that the clean install tree exports exactly one public launch entry point (`full_self_driving.launch.py`), fails closed upon unconfigured hardware selection (`simulation:=false` $\rightarrow$ `HARDWARE_PROFILE_NOT_CONFIGURED`), and contains zero leaked prototype or Offboard dependencies.
3. **Task 15.3: Property 24 (Simulation/Hardware Adapter Invariance)**:
   A formal property-based test (`test/property/property_24_adapter_invariance.cpp` $\rightarrow$ `fsd_property_24_adapter_invariance`) proving that transitioning between simulation and hardware manifests alters *only* declared HAL adapters (transport, camera/TF, payload, telemetry, process, resource), while MissionCoordinator, domain safety rules, persistence protocols, ROS interfaces, and mode ownership remain 100% invariant.
4. **Task 15.4: Authoritative Documentation & Checkpoint G Sign-Off**:
   Update `full_self_driving/MANUAL.md` with Section 16 (Final Acceptance & Acceptance Command Catalog) and confirm full repository regression.

---

## 2. System Architecture & Context

```
+---------------------------------------------------------------------------------------------------+
|                                  FSD MISSION CONTROL & FLIGHT LIFECYCLE                           |
+---------------------------------------------------------------------------------------------------+
|                                                                                                   |
|  [ PREFLIGHT GATEWAY ]                                                                            |
|  - UploadPlanArtifact -> SelectPlanArtifact -> SelectTargetIdentity -> SelectMapScenario         |
|  - PreparePayload (OP_PREPARE_FOR_SORTIE) -> CommitMissionContext (Revision Lock)                |
|                                         |                                                         |
|                                         v                                                         |
|  [ AUTONOMOUS SORTIE LIFECYCLE (14 STAGES) ]                                                      |
|   1. TAKEOFF (10m AGL)                                                                            |
|   2. TRANSIT_IN (Inbound Waypoints)                                                               |
|   3. TARGET ACQUISITION (DIRECT or SEARCH fallback)                                               |
|   4. PRECISION_LAND (Live-Lock Qualification -> Hover/Descent -> Touchdown)                       |
|   5. LANDED_VERIFIED (Ground Disarm & Landing Confirmation)                                       |
|   6. PAYLOAD_OPERATION (Idempotent Release & Named Delivery Feedback)                             |
|   7. TAKEOFF_AFTER_DELIVERY (15m AGL Secondary Climb)                                             |
|   8. TRANSIT_OUT (Outbound Waypoints)                                                             |
|   9. RETURN_STRATEGY (RTL to Locked Sortie Origin Home Base)                                      |
|  10. RETURN_LANDED (EVT_SORTIE_COMPLETED & Safe Standby)                                          |
|  11. EVIDENCE MANIFEST (SHA-256 Digest, Journal, Artifact & Snapshot Export)                      |
|  12. CLEAN MULTI-SORTIE RESTART / RE-ARM READINESS                                                |
|                                                                                                   |
|  [ SAFETY & FAULT BRANCHES (5 SCENARIOS) ]                                                        |
|   A. Stale / Unqualified Target Lock -> Instant Search Fallback / Hover Rejection                 |
|   B. Manual RC / QGC Takeover -> Immediate HOLD & Lower-Priority Interlock                        |
|   C. Emergency Stop -> Immediate FAILSAFE & Absolute Transition Veto                              |
|   D. Persistence Backlog / Storage Reserve Alert -> Real-Time Flight Loop Noninterference         |
|   E. Payload Hardware Fault / Delivery Timeout -> RESULT_FAILURE / RESULT_UNKNOWN Fail-Closed     |
|                                                                                                   |
+---------------------------------------------------------------------------------------------------+
```

---

## 3. Detailed Component Specifications

### 3.1 Task 15.1: End-to-End Simulation Acceptance Suite (`test/acceptance/full_sortie_acceptance.py`)

- **Location**: `full_self_driving/test/acceptance/full_sortie_acceptance.py`
- **Execution Engine**: `pytest` / `unittest` integrated with `ament_cmake` via `ament_add_pytest_test(full_sortie_acceptance)`.
- **Test Structure**:
  1. `TestFullSortieNominalLifecycle`:
     - **Preflight Phase**: Calls Gateway prep services (`UploadPlanArtifact`, `SelectPlanArtifact`, `SelectTargetIdentity`, `SelectMapScenario`, `PreparePayload`). Confirms context enters `LOCKED` status with incremented revision.
     - **Sortie Execution Phase**: Simulates the 14-stage lifecycle sequentially. Verifies transitions:
       - `WAITING_FOR_MODE` $\rightarrow$ `TAKEOFF` (10m altitude setpoint)
       - `TAKEOFF` $\rightarrow$ `TRANSIT_IN` (traversing inbound waypoints)
       - `TRANSIT_IN` $\rightarrow$ `ACQUIRE_TARGET` (evaluating Direct eligibility vs Search fallback)
       - `SEARCH` / `DIRECT` $\rightarrow$ `PRECISION_LAND` (triggered by live target lock)
       - `PRECISION_LAND` $\rightarrow$ `LANDED_VERIFIED` (ground contact & touchdown)
       - `LANDED_VERIFIED` $\rightarrow$ `PAYLOAD_OPERATION` (cargo release execution)
       - `PAYLOAD_OPERATION` $\rightarrow$ `TAKEOFF_AFTER_DELIVERY` (climb to 15m)
       - `TAKEOFF_AFTER_DELIVERY` $\rightarrow$ `TRANSIT_OUT` (traversing outbound waypoints)
       - `TRANSIT_OUT` $\rightarrow$ `RETURN_STRATEGY` (RTL to locked origin base coordinates)
       - `RETURN_STRATEGY` $\rightarrow$ `RETURN_LANDED` (`EVT_SORTIE_COMPLETED`)
     - **Postflight Phase**: Verifies evidence generation, persistence journal durability, and multi-sortie reset capability.
  2. `TestSortieSafetyAndFaultBranches`:
     - **Stale Target Lock**: Feeding expired timestamp (> 0.5s old) rejects lock qualification and keeps vehicle in Search/Hover.
     - **Manual Takeover**: Triggering `DeactivateReason::Other` instantly moves strategy to `HOLD`, disallows autonomous mode transitions, and sets `takeover_active = True`.
     - **Emergency Stop**: Invoking `EmergencyStop.srv` forces `StrategyType::FAILSAFE`, asserts zero further state mutation, and latches emergency stop status.
     - **Persistence Noninterference**: Simulating disk pressure or journal backlogs does not stall or delay the flight control loop.
     - **Payload Fault Handling**: Injected delivery timeout or hardware fault produces explicit `RESULT_UNKNOWN` or `RESULT_FAILURE` and enters safe ReturnStrategy without unbounded retries.

---

### 3.2 Task 15.2: Clean Install, One-Launch Ownership & Isolated Workspace Regression

- **Location**: `full_self_driving/test/launch/launch_boundary_test.py` and workspace packaging.
- **Verification Criteria**:
  1. **Single Public Launch Entry Point**: Inspect `${CMAKE_INSTALL_PREFIX}/share/full_self_driving/launch/` to guarantee that *exactly one* launch file (`full_self_driving.launch.py`) exists.
  2. **Fail-Closed Hardware Branch**: Executing `ros2 launch full_self_driving full_self_driving.launch.py simulation:=false` without a signed hardware manifest fails with exit code $\ne 0$ and error output containing `HARDWARE_PROFILE_NOT_CONFIGURED`.
  3. **Zero Leaked Prototype/Offboard Dependencies**: Static AST inspection confirms no references to `px4_roscon_25`, `transit_in`, `aruco_tracker`, `OffboardControlMode`, or raw `/fmu/in/*` actuator topics in the install artifact.
  4. **Isolated Workspace Regression**: Clean build and test run in container passes 100% of CTest targets.

---

### 3.3 Task 15.3: Property 24 (Simulation/Hardware Adapter Invariance)

- **Location**: `full_self_driving/test/property/property_24_adapter_invariance.cpp`
- **Target Name**: `fsd_property_24_adapter_invariance`
- **Validates**: Requirement 1.1, Safety Property 24.
- **Invariant Statements**:
  1. **Domain Safety Invariance**: Switching configuration profile from `simulation` to `hardware` preserves identical `EngineeringConfig` validation rules, minimum battery thresholds, timeout constants, geofence bounds, and transition preconditions.
  2. **Mission Coordinator & Mode Ownership Invariance**: The state transition matrix, takeover precedence, emergency stop interlock, and single `FullSelfDrivingMode` / `FullSelfDrivingModeExecutor` ownership hierarchy remain identical regardless of profile.
  3. **ROS Interface & Contract Invariance**: All 20 `.msg` and 10 `.srv` schemas, topic names, QoS profiles, message bounds, and service request/response semantics are completely invariant.
  4. **Persistence & Recovery Invariance**: Journal serialization schemas (`journal.schema.json`), snapshot checksum algorithms (SHA-256), atomic file write protocols, and recovery reconciliation state machines are completely invariant.
  5. **Declared Adapter Exclusivity**: The *only* components that differ between profiles are declared HAL adapters:
     - Transport: MicroXRCE-DDS UDP client vs Serial/UART bridge
     - Camera/TF: Gazebo camera bridge vs V4L2/libcamera hardware driver
     - Payload: `SimulationPayloadAdapter` vs GPIO/PWM hardware actuator
     - Telemetry: SITL MAVLink UDP vs Telemetry Radio UART
     - Resource paths: Simulation model meshes vs physical sensor device paths (`/dev/video*`, `/dev/tty*`)

---

### 3.4 Task 15.4: Authoritative Documentation Update (`MANUAL.md` Section 16)

- **Location**: `full_self_driving/MANUAL.md`
- **Section 16: Final Acceptance & Acceptance Command Catalog**:
  - Full description of Checkpoint G verification procedures.
  - Acceptance command catalog for SITL simulation, automated CTest suites, and clean-install verification.
  - Complete matrix of Safety Properties 1 through 26 and their corresponding test executables.

---

## 4. Verification & Validation Plan

| Test File | Test Suite / Target | Purpose | Pass Criteria |
| :--- | :--- | :--- | :--- |
| `test/acceptance/full_sortie_acceptance.py` | `full_sortie_acceptance` | End-to-end autonomous sortie lifecycle (14 stages) and 5 safety fault branches | 100% pass, all assertions green |
| `test/launch/launch_boundary_test.py` | `launch_boundary_test` | One-launch ownership and fail-closed hardware manifest gate | `simulation:=false` fails closed with `HARDWARE_PROFILE_NOT_CONFIGURED` |
| `test/property/property_24_adapter_invariance.cpp` | `fsd_property_24_adapter_invariance` | Formal proof of simulation/hardware adapter invariance (Property 24) | 100% pass across all property assertions |
| `test/security/production_boundary_scan.py` | `production_boundary_scan` | Repository-wide boundary and forbidden symbol scanner | Zero forbidden prototype or offboard symbols |
| `colcon test` | All 43+ test targets | Full repository test suite | 100% pass (0 failures, 0 errors, 0 skipped) |

---

## 5. Risk Analysis & Mitigation

1. **Risk**: Brittle timing dependencies in end-to-end Python acceptance tests causing sporadic timeouts.  
   - **Mitigation**: Implement a deterministic mock & event-driven client harness that queries status synchronously and verifies transitions without arbitrary `time.sleep()` calls.
2. **Risk**: Hardware manifest gate bypassing if parameters are misparsed.  
   - **Mitigation**: Add explicit unit and integration test assertions verifying that omitting `hardware_manifest` when `simulation:=false` fails immediately during launch evaluation before any nodes are spawned.
3. **Risk**: Scope creep into actual Raspberry Pi 4 hardware bringup.  
   - **Mitigation**: Strictly respect the task boundary: Task 15 proves the *invariance contract* and *fail-closed gate*; actual physical Pi 4 bringup remains deferred behind Task 16.1.
