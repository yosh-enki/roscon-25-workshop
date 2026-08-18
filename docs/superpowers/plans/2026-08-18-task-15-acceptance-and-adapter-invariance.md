# Task 15: End-to-End Simulation Acceptance & Adapter-Invariance Proof Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the complete Task 15 end-to-end simulation acceptance suite, clean install and single-launch boundary verification, and Safety Property 24 adapter-invariance proof, achieving Checkpoint G sign-off with 100% test pass.

**Architecture:** 
- `test/acceptance/full_sortie_acceptance.py`: Deterministic ROS 2 client acceptance harness exercising the full 14-stage autonomous sortie lifecycle and 5 safety fault branches.
- `test/launch/launch_boundary_test.py` & `test/acceptance/test_clean_install_boundary.py`: Verifies single-launch ownership (`full_self_driving.launch.py`) and fail-closed hardware manifest gate (`simulation:=false` $\rightarrow$ `HARDWARE_PROFILE_NOT_CONFIGURED`).
- `test/property/property_24_adapter_invariance.cpp`: C++ GTest property proof verifying invariance of domain safety rules, coordinator transitions, ROS contracts, persistence protocols, and mode ownership across simulation vs hardware profiles.
- `full_self_driving/MANUAL.md`: Section 16 authoritative acceptance and operational catalog.

**Tech Stack:** ROS 2 Humble, C++17, Python 3.10, `rclcpp`, `rclpy`, `px4_ros2_cpp`, `ament_cmake`, `pytest`, GoogleTest, OpenSSL SHA-256.

## Global Constraints

- Exclusive registered `px4_ros2_cpp` ModeBase/ModeExecutor control; zero Offboard or raw `/fmu/in/*` actuator control topics.
- Exactly one public launch file: `full_self_driving.launch.py`.
- Fail-closed hardware profile gate when `simulation:=false` without approved manifest.
- Zero prototype dependencies (`px4_roscon_25`, `transit_in`, `aruco_tracker`, etc.).
- Bounded ROS message definitions (`string<=N`, `sequence[<=N]`).
- 100% test pass across all unit, integration, property, security, and acceptance suites.

---

### Task 1: Clean Baseline & Task 14 Security Hardening Commit

**Files:**
- Modify: `full_self_driving/CMakeLists.txt:1210-1296`
- Modify: `full_self_driving/MANUAL.md:1350-1440`
- Modify: `full_self_driving/src/gateway/fsd_gateway.cpp`
- Test: `full_self_driving/test/integration/resource_failure_test.cpp`
- Test: `full_self_driving/test/property/property_25_observability_noninterference.cpp`
- Test: `full_self_driving/test/security/gateway_security_test.cpp`
- Test: `full_self_driving/test/security/production_boundary_scan.py`

**Interfaces:**
- Consumes: Task 13 SROS2 keystore, Task 14 security tests and boundary scanner.
- Produces: Clean 295 passing tests baseline in container.

- [ ] **Step 1: Verify all 295 baseline tests pass**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon test --packages-select full_self_driving && colcon test-result --all --verbose"
```
Expected: PASS (295 tests, 0 errors, 0 failures, 0 skipped).

- [ ] **Step 2: Commit Task 14 test fixes and baseline**

```bash
git add full_self_driving/CMakeLists.txt full_self_driving/MANUAL.md full_self_driving/src/gateway/fsd_gateway.cpp full_self_driving/src/payload/payload_controller.cpp full_self_driving/test/integration/resource_failure_test.cpp full_self_driving/test/property/property_25_observability_noninterference.cpp full_self_driving/test/security/forbidden_dependency_scan.py full_self_driving/test/security/gateway_security_test.cpp full_self_driving/test/security/production_boundary_scan.py
git commit -m "fix(security): resolve Task 14 test harness interfaces and achieve 295 test green baseline"
```

---

### Task 2: End-to-End Simulation Acceptance Suite (Task 15.1)

**Files:**
- Create: `full_self_driving/test/acceptance/full_sortie_acceptance.py`
- Modify: `full_self_driving/CMakeLists.txt:1290-1300`

**Interfaces:**
- Consumes: `full_self_driving` ROS 2 services (`srv/UploadPlanArtifact`, `srv/SelectPlanArtifact`, `srv/SelectTargetIdentity`, `srv/SelectMapScenario`, `srv/PreparePayload`, `srv/CommitMissionContext`, `srv/EmergencyStop`, `srv/FullSelfDrivingCommand`), messages (`msg/FullSelfDrivingState`, `msg/MissionContext`, `msg/LiveTargetLock`, `msg/VehicleTelemetry`), domain strategies and coordinator.
- Produces: Deterministic acceptance test suite validating 14-stage autonomous sortie and 5 fault branches.

- [ ] **Step 1: Write the failing acceptance test**

Create `full_self_driving/test/acceptance/full_sortie_acceptance.py` with:
- `TestFullSortieNominalLifecycle`: Exercises full 14-stage autonomous sortie flow:
  1. Gateway Preflight Preparation (`UploadPlanArtifact`, `SelectPlanArtifact`, `SelectTargetIdentity`, `SelectMapScenario`, `PreparePayload`).
  2. Mission Context Commit & Lock (`CommitMissionContext`).
  3. Sortie Takeoff (`TAKEOFF` 10m AGL).
  4. TransitIn route progression.
  5. Target Acquisition (Direct vs Search fallback).
  6. Live-Lock PrecisionLand (LiveTargetLock qualification, hover/descent gates).
  7. Landing verification (`LANDED_VERIFIED`, ground disarm).
  8. Payload operation (`OP_RELEASE_CARGO`, feedback confirmation).
  9. Takeoff after delivery (`TAKEOFF_AFTER_DELIVERY` 15m AGL).
  10. TransitOut route progression.
  11. ReturnStrategy (RTL to locked origin / home base).
  12. Return landed (`RETURN_LANDED`, `EVT_SORTIE_COMPLETED`).
  13. Evidence manifest export & SHA-256 validation.
  14. Disarm & clean multi-sortie re-arm readiness.
- `TestSortieSafetyAndFaultBranches`:
  1. Stale / unqualified lock rejection.
  2. Manual RC/QGC takeover overriding autonomy into `HOLD`.
  3. Emergency stop fail-closed transition into `FAILSAFE`.
  4. Persistence reserve warning & heavy logging noninterference.
  5. In-flight payload timeout & hardware failure fail-closed handling.

- [ ] **Step 2: Register `full_sortie_acceptance` in `CMakeLists.txt`**

Add `ament_add_pytest_test(full_sortie_acceptance test/acceptance/full_sortie_acceptance.py)` under `if(BUILD_TESTING)` in `full_self_driving/CMakeLists.txt`.

- [ ] **Step 3: Run acceptance test in docker container**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON && pytest-3 src/roscon-25-workshop/full_self_driving/test/acceptance/full_sortie_acceptance.py -v"
```
Expected: PASS with 100% assertions green.

- [ ] **Step 4: Commit Task 15.1**

```bash
git add full_self_driving/CMakeLists.txt full_self_driving/test/acceptance/full_sortie_acceptance.py
git commit -m "feat(acceptance): implement Task 15.1 full autonomous sortie acceptance test suite"
```

---

### Task 3: Clean Install, One-Launch Ownership & Isolated Workspace Regression (Task 15.2)

**Files:**
- Modify: `full_self_driving/test/launch/launch_boundary_test.py`
- Modify: `full_self_driving/launch/full_self_driving.launch.py`

**Interfaces:**
- Consumes: ROS 2 launch system, install tree file hierarchy, launch argument validation.
- Produces: Launch validation verifying exactly one public launch entry point and fail-closed hardware manifest gate.

- [ ] **Step 1: Write the failing launch and install tests in `launch_boundary_test.py`**

Add tests:
- `test_single_public_launch_entry_point`: Verifies that only `full_self_driving.launch.py` is installed in the package `share/full_self_driving/launch` directory.
- `test_hardware_profile_deferral_fails_closed`: Verifies `simulation:=false` without approved hardware manifest fails with exit code $\ne 0$ and `HARDWARE_PROFILE_NOT_CONFIGURED`.
- `test_no_prototype_or_raw_offboard_dependencies_in_installed_share`: Verifies that installed launch and resource trees contain zero references to `px4_roscon_25` or `/fmu/in/*`.

- [ ] **Step 2: Run `launch_boundary_test` in container**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && pytest-3 src/roscon-25-workshop/full_self_driving/test/launch/launch_boundary_test.py -v"
```
Expected: PASS (all tests pass).

- [ ] **Step 3: Commit Task 15.2**

```bash
git add full_self_driving/launch/full_self_driving.launch.py full_self_driving/test/launch/launch_boundary_test.py
git commit -m "feat(launch): implement Task 15.2 clean install and single-launch fail-closed verification"
```

---

### Task 4: Safety Property 24 Simulation/Hardware Adapter Invariance (Task 15.3)

**Files:**
- Create: `full_self_driving/test/property/property_24_adapter_invariance.cpp`
- Modify: `full_self_driving/CMakeLists.txt:1220-1240`

**Interfaces:**
- Consumes: `EngineeringConfig`, `MissionCoordinator`, `MissionContext`, `PersistenceManager`, `PayloadController`, `SimulationPayloadAdapter`, ROS 2 interfaces, and `FullSelfDrivingMode`.
- Produces: Formal property test executable `fsd_property_24_adapter_invariance`.

- [ ] **Step 1: Write `property_24_adapter_invariance.cpp`**

Implement GoogleTest test cases validating Property 24:
1. `Property24_DomainSafetyRulesInvariantAcrossProfiles`: Proves domain config validation rules, minimum battery thresholds, timeout constants, geofence bounds, and transition preconditions are identical.
2. `Property24_CoordinatorStateMachineInvariantAcrossProfiles`: Proves coordinator transition matrix, takeover precedence, emergency stop interlock, and mode ownership remain identical regardless of profile.
3. `Property24_RosContractAndPersistenceInvariantAcrossProfiles`: Proves journal serialization schemas (`journal.schema.json`), snapshot checksum algorithms (SHA-256), and recovery reconciliation are completely invariant.
4. `Property24_OnlyDeclaredAdaptersDiffer`: Proves that switching profiles only modifies declared HAL adapters (transport, camera/TF, payload, telemetry, process, resource).

- [ ] **Step 2: Register `fsd_property_24_adapter_invariance` in `CMakeLists.txt`**

```cmake
ament_add_gtest(fsd_property_24_adapter_invariance
  test/property/property_24_adapter_invariance.cpp
)
target_include_directories(fsd_property_24_adapter_invariance PRIVATE
  include
  src
)
target_link_libraries(fsd_property_24_adapter_invariance
  fsd_flight_core
  fsd_payload_core
  fsd_adapters_core
  fsd_persistence_core
  fsd_domain_core
  fsd_registry_core
  OpenSSL::Crypto
  yaml-cpp
  "${cpp_typesupport_target}"
)
ament_target_dependencies(fsd_property_24_adapter_invariance
  rclcpp
  px4_ros2_cpp
  px4_msgs
  geometry_msgs
  builtin_interfaces
)
```

- [ ] **Step 3: Build and run `fsd_property_24_adapter_invariance`**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/full_self_driving -R fsd_property_24_adapter_invariance --output-on-failure"
```
Expected: PASS (all property assertions green).

- [ ] **Step 4: Commit Task 15.3**

```bash
git add full_self_driving/CMakeLists.txt full_self_driving/test/property/property_24_adapter_invariance.cpp
git commit -m "feat(property): implement Property 24 simulation/hardware adapter-invariance test"
```

---

### Task 5: Authoritative Documentation, Manual Catalog & Checkpoint G Sign-Off (Task 15.4)

**Files:**
- Modify: `full_self_driving/MANUAL.md:1430-1500`

**Interfaces:**
- Consumes: Final test results, command catalogs, property catalog (Properties 1–26), Checkpoint G criteria.
- Produces: Complete Section 16 in `full_self_driving/MANUAL.md`.

- [ ] **Step 1: Write Section 16 in `full_self_driving/MANUAL.md`**

Update `full_self_driving/MANUAL.md` with:
- **Section 16: Final Acceptance & Acceptance Command Catalog**:
  - Live simulation acceptance execution instructions (`ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:=kmitl_airfield`).
  - Automated acceptance test commands (`pytest-3 full_sortie_acceptance.py`, `ctest -R fsd_property_24`).
  - Complete Safety Property verification table (Properties 1 through 26).
  - Checkpoint G sign-off declaration and evidence validation.

- [ ] **Step 2: Run full regression across the entire test suite**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select full_self_driving && colcon test-result --all --verbose"
```
Expected: 100% test pass (0 failures, 0 errors, 0 skipped).

- [ ] **Step 3: Commit Task 15.4 and Checkpoint G**

```bash
git add full_self_driving/MANUAL.md
git commit -m "docs(manual): complete Section 16 Final Acceptance and Checkpoint G documentation"
```
