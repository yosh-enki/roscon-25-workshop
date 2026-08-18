# Task 16: Defer Raspberry Pi 4 Hardware Bringup Behind an Explicit Manifest Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the formal hardware manifest schema, C++ hardware manifest validator library & CLI probe, clean hardware HAL interface stubs, single launch fail-closed gate integration, and the authoritative Hardware Installation & Provisioning Manual (Section 17) for Task 16.1.

**Architecture:**
- `simulation/manifests/hardware_schema.yaml`: Declarative YAML schema for physical FMU transport, V4L2 camera, GPIO/PWM payload, calibration SHA-256 hashes, SROS2 keystores, and cryptographic approval records.
- `src/launch/hardware_manifest_validator.hpp` / `.cpp` & `fsd_hardware_manifest_validator`: Robust C++ validation engine and CLI tool checking schema validity, device node paths, permissions, OpenSSL SHA-256 calibration checksums, and approval signatures.
- `src/adapters/hardware_fmu_adapter.hpp`, `src/adapters/hardware_camera_adapter.hpp`, `src/payload/hardware_payload_adapter.hpp`: Explicit C++ HAL interface stubs returning deferred health if instantiated, with zero fake mocks or Gazebo fallbacks.
- `launch/full_self_driving.launch.py` & `test/launch/launch_boundary_test.py`: Integrates `hardware_manifest` argument and enforces `simulation:=false` fail-closed gate (`HARDWARE_PROFILE_NOT_CONFIGURED`).
- `full_self_driving/MANUAL.md`: Section 17 complete Hardware Installation & Provisioning Manual.

**Tech Stack:** ROS 2 Humble, C++17, Python 3.10, `yaml-cpp`, OpenSSL (SHA-256), GoogleTest, `pytest`, `ament_cmake`.

## Global Constraints

- Exclusive registered `px4_ros2_cpp` ModeBase/ModeExecutor control; zero Offboard or raw `/fmu/in/*` actuator control topics.
- Exactly one public launch file: `full_self_driving.launch.py`.
- Fail-closed hardware profile gate when `simulation:=false` without approved manifest.
- Zero prototype dependencies (`px4_roscon_25`, `transit_in`, `aruco_tracker`, etc.).
- Bounded ROS message definitions (`string<=N`, `sequence[<=N]`).
- 100% test pass across all unit, integration, property, security, and acceptance suites.

---

### Task 1: Hardware Manifest Schema & Test Manifest Fixtures

**Files:**
- Create: `full_self_driving/simulation/manifests/hardware_schema.yaml`
- Create: `full_self_driving/config/camera_calibrations/imx219_720p.yaml`
- Create: `full_self_driving/test/fixtures/manifests/valid_hardware_manifest.yaml`
- Create: `full_self_driving/test/fixtures/manifests/unapproved_hardware_manifest.yaml`
- Create: `full_self_driving/test/fixtures/manifests/tampered_calibration_manifest.yaml`
- Create: `full_self_driving/test/fixtures/manifests/missing_fmu_manifest.yaml`
- Create: `full_self_driving/test/fixtures/manifests/invalid_adapter_id_manifest.yaml`

**Interfaces:**
- Consumes: YAML schema format and SHA-256 checksums.
- Produces: Formal hardware schema and reference test fixtures for C++ validator and launch boundary testing.

- [ ] **Step 1: Create `simulation/manifests/hardware_schema.yaml`**

Write `full_self_driving/simulation/manifests/hardware_schema.yaml` with required fields (`profile`, `manifest_version`, `approval`, `fmu_transport`, `camera`, `payload`, `security`, `system_resources`).

- [ ] **Step 2: Create reference camera calibration `config/camera_calibrations/imx219_720p.yaml`**

Create `full_self_driving/config/camera_calibrations/imx219_720p.yaml` containing camera matrix, distortion coefficients, image dimensions (1280x720), and compute its SHA-256 checksum.

- [ ] **Step 3: Create test manifest fixtures in `test/fixtures/manifests/`**

Create:
- `valid_hardware_manifest.yaml` with `approved: true` and matching calibration SHA-256.
- `unapproved_hardware_manifest.yaml` with `approved: false`.
- `tampered_calibration_manifest.yaml` with mismatched SHA-256 hash.
- `missing_fmu_manifest.yaml` missing `fmu_transport` section.
- `invalid_adapter_id_manifest.yaml` with unknown adapter ID.

- [ ] **Step 4: Commit Task 1 files**

```bash
git add full_self_driving/simulation/manifests/hardware_schema.yaml full_self_driving/config/camera_calibrations/imx219_720p.yaml full_self_driving/test/fixtures/manifests/
git commit -m "feat(manifest): add hardware schema and test manifest fixtures"
```

---

### Task 2: C++ Hardware Manifest Validator Library & CLI Executable

**Files:**
- Create: `full_self_driving/src/launch/hardware_manifest_validator.hpp`
- Create: `full_self_driving/src/launch/hardware_manifest_validator.cpp`
- Create: `full_self_driving/src/launch/fsd_hardware_manifest_validator_main.cpp`
- Create: `full_self_driving/test/launch/hardware_manifest_validator_test.cpp`
- Modify: `full_self_driving/CMakeLists.txt`

**Interfaces:**
- Consumes: `yaml-cpp`, OpenSSL (`EVP_sha256`), POSIX `access()`, `stat()`.
- Produces: `fsd_hardware_manifest_validator_lib` library, `fsd_hardware_manifest_validator` CLI executable, and `hardware_manifest_validator_test` GTest target.

- [ ] **Step 1: Write the failing test `test/launch/hardware_manifest_validator_test.cpp`**

Write GTest test cases testing:
- `ValidManifest`: Expect `result.is_valid == true` and `result.status == ValidationStatus::VALID`.
- `UnapprovedManifest`: Expect `result.is_valid == false` and `result.status == ValidationStatus::APPROVAL_DEFERRED_OR_MISSING`.
- `TamperedCalibration`: Expect `result.is_valid == false` and `result.status == ValidationStatus::CALIBRATION_CHECKSUM_MISMATCH`.
- `MissingFmuTransport`: Expect `result.is_valid == false` and `result.status == ValidationStatus::SCHEMA_MISSING_REQUIRED_FIELD`.
- `InvalidAdapterId`: Expect `result.is_valid == false` and `result.status == ValidationStatus::ADAPTER_ID_MISMATCH`.

- [ ] **Step 2: Implement `src/launch/hardware_manifest_validator.hpp` and `.cpp`**

Implement `full_self_driving::launch::HardwareManifestValidator` with:
- `validate_file(manifest_path, check_physical_devices, base_dir)`
- `validate_yaml(root, check_physical_devices, base_dir)`
- `compute_file_sha256(file_path)` using OpenSSL EVP.

- [ ] **Step 3: Implement `src/launch/fsd_hardware_manifest_validator_main.cpp`**

Implement the standalone CLI binary with `--manifest <path>`, `--check-devices`, and `--json` flags.

- [ ] **Step 4: Update `full_self_driving/CMakeLists.txt`**

Add targets:
- `add_library(fsd_hardware_manifest_validator_lib src/launch/hardware_manifest_validator.cpp)`
- `add_executable(fsd_hardware_manifest_validator src/launch/fsd_hardware_manifest_validator_main.cpp)`
- `ament_add_gtest(hardware_manifest_validator_test test/launch/hardware_manifest_validator_test.cpp)`
- Install `fsd_hardware_manifest_validator` into `lib/${PROJECT_NAME}`.

- [ ] **Step 5: Run tests in Docker container and verify PASS**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON && ctest --test-dir build/full_self_driving -R hardware_manifest_validator_test --output-on-failure"
```

- [ ] **Step 6: Commit Task 2 files**

```bash
git add full_self_driving/src/launch/ full_self_driving/test/launch/hardware_manifest_validator_test.cpp full_self_driving/CMakeLists.txt
git commit -m "feat(validator): implement C++ hardware manifest validator library, CLI tool, and unit tests"
```

---

### Task 3: Clean Hardware HAL Interface Stubs

**Files:**
- Create: `full_self_driving/src/adapters/hardware_fmu_adapter.hpp`
- Create: `full_self_driving/src/adapters/hardware_fmu_adapter.cpp`
- Create: `full_self_driving/src/adapters/hardware_camera_adapter.hpp`
- Create: `full_self_driving/src/adapters/hardware_camera_adapter.cpp`
- Create: `full_self_driving/src/payload/hardware_payload_adapter.hpp`
- Create: `full_self_driving/src/payload/hardware_payload_adapter.cpp`
- Modify: `full_self_driving/CMakeLists.txt`

**Interfaces:**
- Consumes: `full_self_driving::payload::PayloadAdapter`, `HardwareManifest`.
- Produces: Explicit C++ HAL interface stubs for physical FMU transport (`px4_hardware_uart_serial`), V4L2 camera (`v4l2_hardware_camera`), and GPIO/PWM payload (`gpio_pwm_payload_actuator`).

- [ ] **Step 1: Implement `src/adapters/hardware_fmu_adapter.hpp` and `.cpp`**

Define `HardwareFmuAdapter` with UART path, baud rate, flow control, and fail-closed state (`is_connected() == false`, `is_healthy() == false` with deferral reason).

- [ ] **Step 2: Implement `src/adapters/hardware_camera_adapter.hpp` and `.cpp`**

Define `HardwareCameraAdapter` with device path, resolution, calibration hash, and fail-closed state (`is_capturing() == false`).

- [ ] **Step 3: Implement `src/payload/hardware_payload_adapter.hpp` and `.cpp`**

Implement `HardwarePayloadAdapter` inheriting from `PayloadAdapter`:
- `get_adapter_id()` returns `"gpio_pwm_payload_actuator"`.
- `is_healthy()` returns `false`.
- `execute_command(...)` returns `false` and sets `status.result = full_self_driving::msg::PayloadStatus::RESULT_REJECTED`.

- [ ] **Step 4: Update `CMakeLists.txt` to compile adapters**

Add `src/adapters/hardware_fmu_adapter.cpp`, `src/adapters/hardware_camera_adapter.cpp`, and `src/payload/hardware_payload_adapter.cpp` to `fsd_domain_core` / `fsd_adapters` library targets.

- [ ] **Step 5: Build and verify no compilation regressions**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON"
```

- [ ] **Step 6: Commit Task 3 files**

```bash
git add full_self_driving/src/adapters/ full_self_driving/src/payload/hardware_payload_adapter.* full_self_driving/CMakeLists.txt
git commit -m "feat(hal): implement clean hardware HAL interface stubs for FMU, camera, and payload"
```

---

### Task 4: Single Launch Entry Point Integration & Fail-Closed Gate Verification

**Files:**
- Modify: `full_self_driving/launch/full_self_driving.launch.py:40-55`
- Modify: `full_self_driving/test/launch/launch_boundary_test.py:28-40`
- Modify: `full_self_driving/test/launch/launch_manifest_test.cpp`

**Interfaces:**
- Consumes: `hardware_manifest` launch configuration.
- Produces: Verified single launch fail-closed gate behavior.

- [ ] **Step 1: Update `full_self_driving.launch.py`**

- Add `DeclareLaunchArgument("hardware_manifest", default_value="", description="Path to approved hardware manifest")`.
- In `launch_setup()`:
  - If `not simulation`:
    - Check if `hardware_manifest` is empty or invalid.
    - If empty $\rightarrow$ log error and raise `RuntimeError("HARDWARE_PROFILE_NOT_CONFIGURED")`.
    - If non-empty $\rightarrow$ invoke validator logic. If unapproved or validation fails $\rightarrow$ log error and raise `RuntimeError("HARDWARE_PROFILE_NOT_CONFIGURED: Hardware bringup is deferred pending validated profile.")`.

- [ ] **Step 2: Update `test/launch/launch_boundary_test.py` and `test/launch/launch_manifest_test.cpp`**

- Test `simulation:=false` without manifest $\rightarrow$ fails with `HARDWARE_PROFILE_NOT_CONFIGURED`.
- Test `simulation:=false hardware_manifest:=unapproved_hardware_manifest.yaml` $\rightarrow$ fails with `HARDWARE_PROFILE_NOT_CONFIGURED`.
- Verify `hardware_schema.yaml` exists in package share.

- [ ] **Step 3: Run launch boundary and manifest tests**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DBUILD_TESTING=ON && pytest-3 src/roscon-25-workshop/full_self_driving/test/launch/launch_boundary_test.py -v && ctest --test-dir build/full_self_driving -R launch_manifest_test --output-on-failure"
```

- [ ] **Step 4: Commit Task 4 files**

```bash
git add full_self_driving/launch/full_self_driving.launch.py full_self_driving/test/launch/
git commit -m "feat(launch): integrate hardware manifest gate into single launch entry point"
```

---

### Task 5: Hardware Installation & Provisioning Manual (`MANUAL.md` Section 17) & Task Tracking

**Files:**
- Modify: `full_self_driving/MANUAL.md`
- Modify: `spec/tasks.md:670-680`

**Interfaces:**
- Consumes: Task 16.1 implementation details, hardware specifications, wiring pinouts, `udev` rules, SROS2 commands, and manifest sign-off protocols.
- Produces: Complete Section 17 in `full_self_driving/MANUAL.md` and marked Task 16.1 in `spec/tasks.md`.

- [ ] **Step 1: Write Section 17 in `full_self_driving/MANUAL.md`**

Add Section 17 detailing:
- 17.1 Hardware Architecture & Bill of Materials (BOM)
- 17.2 Electrical & Physical Interconnects (Pixhawk 6C TELEM2 $\leftrightarrow$ RPi4 UART, Camera CSI/USB, GPIO 18 PWM)
- 17.3 Companion Computer OS, Toolchain & Permissions (`dialout`, `video`, `gpio`)
- 17.4 Deterministic `udev` Rules Setup (`/etc/udev/rules.d/99-fsd-hardware.rules`)
- 17.5 Camera Intrinsic/Extrinsic Calibration & SHA-256 Hashing Procedure
- 17.6 SROS2 Keystore Deployment & Permission Profile
- 17.7 Hardware Manifest Approval & Safety Sign-Off Gate
- 17.8 Operational Command Catalog for Hardware Manifest Verification (`fsd_hardware_manifest_validator`)

- [ ] **Step 2: Update `spec/tasks.md`**

Mark Task 16.1 as `[x]`.

- [ ] **Step 3: Commit Task 5 files**

```bash
git add full_self_driving/MANUAL.md spec/tasks.md
git commit -m "docs(manual): add Section 17 Hardware Installation Manual and mark Task 16.1 complete"
```

---

### Task 6: Clean Workspace Build & Full Regression Verification Across All Test Suites

**Files:**
- Test: All 45+ test targets in `full_self_driving`.

- [ ] **Step 1: Clean build the package**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && colcon build --symlink-install --packages-select full_self_driving --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON"
```
Expected: Build successfully completes with 0 errors.

- [ ] **Step 2: Run complete CTest and Pytest regression suites**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash && colcon test --packages-select full_self_driving --event-handlers console_direct+ && colcon test-result --all --verbose"
```
Expected: 100% test pass across all unit, integration, property, security, launch, and acceptance test suites (0 failures, 0 errors).

- [ ] **Step 3: Run repository-wide boundary and security scans**

```bash
docker exec px4-roscon-25 bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && pytest-3 src/roscon-25-workshop/full_self_driving/test/security/production_boundary_scan.py -v && pytest-3 src/roscon-25-workshop/full_self_driving/test/security/forbidden_dependency_scan.py -v && pytest-3 src/roscon-25-workshop/full_self_driving/test/security/security_policy_enforcement_test.py -v"
```
Expected: All security scans pass with zero violations.

- [ ] **Step 4: Commit final verification baseline if needed**

```bash
git status
```
