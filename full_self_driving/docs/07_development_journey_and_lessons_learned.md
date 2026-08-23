# Module 07: Development Journey & Lessons Learned

This document provides a comprehensive historical analysis of the engineering journey behind the `full_self_driving` package. It captures architectural evolutions from initial prototype concepts to production robustness, followed by deep-dive post-mortems of critical real-world bugs encountered and resolved throughout Git history.

---

## 1. The Engineering Journey (Phase-by-Phase)

```
2026-08-17                      2026-08-18                      2026-08-20                      2026-08-23
┌───────────────────────────────┬───────────────────────────────┬───────────────────────────────┬───────────────────────────────┐
│ Phase 1: Foundation           │ Phase 2: Autonomous Sortie    │ Phase 3: Hardware & UI        │ Phase 4: Production Hardening │
│ • Single Launch File          │ • 8 Flight Strategies         │ • Native PX4 Gripper          │ • Hard Arming Rejection Gates │
│ • Gazebo + PX4 SITL           │ • Precision Land 5-Phase Loop │ • ESP32 Servo HITL Testbed    │ • Multi-Sortie Reset Fixes    │
│ • OpenCV solvePnP             │ • Scoped Pad Registry         │ • Foxglove Aerospace HUD      │ • Unified Parameter Catalog   │
│ • px4_ros2_cpp Registered Mode│ • SROS2 DDS Security Hardening│ • Plan Manager Extension      │ • 100% Test Pass Baseline     │
└───────────────────────────────┴───────────────────────────────┴───────────────────────────────┴───────────────────────────────┘
```

### Phase 1: Foundation & Single Launch Invariant (Tasks 1–7)
- **Goal**: Eliminate fragmented bash scripts (`run_world.sh`, multi-terminal `MicroXRCEAgent`) and brittle Offboard mode hacks.
- **Milestone**: Created [`launch/full_self_driving.launch.py`](file:///home/yosh/roscon-25-workshop/full_self_driving/launch/full_self_driving.launch.py) orchestrating Gazebo Harmonic, PX4 SITL, MicroXRCEAgent, ROS-GZ bridges, and TF2 in a single supervised lifecycle.
- **Milestone**: Replaced legacy Offboard setpoint streaming with native `px4_ros2_cpp` Mode registration (`FullSelfDrivingMode`), achieving first-class autopilot integration.

### Phase 2: Full Autonomous Sortie Engine (Tasks 8–16)
- **Goal**: Complete autonomous delivery cycle (Takeoff $\rightarrow$ TransitIn $\rightarrow$ Direct/Search $\rightarrow$ Precision Land $\rightarrow$ Payload $\rightarrow$ TransitOut $\rightarrow$ RTL).
- **Milestone**: Designed vision-guided touchdown controller with coast-to-stop `HOVER_BRAKE` and closed-loop velocity setpoint streaming.
- **Milestone**: Gated physical Raspberry Pi 4 bringup with the Hardware Manifest Validator (`HARDWARE_PROFILE_NOT_CONFIGURED`).

### Phase 3: Hardware-in-the-Loop & Aerospace Mission Control
- **Goal**: Real physical actuator validation and modern ground station visualization.
- **Milestone**: Implemented the ESP32 Servo HITL testbed with custom Arduino firmware, DTR/RTS auto-reset draining, and NVIDIA GPU passthrough in Docker.
- **Milestone**: Developed the 3-column Foxglove Studio Aerospace HUD and packaged `.foxe` panel extension.

### Phase 4: Production Hardening & Multi-Sortie Safety
- **Goal**: Iron out edge cases in continuous multi-sortie operations and flight state transitions.
- **Milestone**: Unified all configuration parameters into [`config/fsd_parameters.yaml`](file:///home/yosh/roscon-25-workshop/full_self_driving/config/fsd_parameters.yaml).
- **Milestone**: Enforced hard arming rejection gates and decoupled ground disarms from manual takeover alarms.

---

## 2. Deep-Dive Bug Post-Mortems & Lessons Learned

### Bug 1: Transit-In Hold & Takeover Latching Bug
- **Commit**: `8bf5e0a` (*fix(flight): resolve stuck transit-in hold, takeover latching, and direct acquisition*)
- **Symptom**: When manual takeover was triggered and then reset by the operator, the drone became permanently stuck in `HOLD` mode and would not resume transit.
- **Root Cause**: `MissionCoordinator::reset_takeover()` cleared the boolean flag `takeover_active_ = false`, but did not evaluate or reinstate the previously active strategy in the state machine. The FSM remained pointing to `HOLD`.
- **Solution**: Added strategy re-evaluation logic in `reset_takeover()` to transition cleanly from `HOLD` back into the appropriate flight strategy depending on flight phase.
- **Architectural Lesson**: State flags and FSM active states must be synchronized atomically. Resetting an error flag must always define a valid target state transition.

---

### Bug 2: Sortie Origin Home Base Coordinate Lock
- **Commit**: `f15551e` (*fix(flight): lock sortie origin home base coordinates at initial arm to fix RTL return target*)
- **Symptom**: During multi-pad delivery sorties, when the drone completed landing at an intermediate pad and re-armed for the return flight, `ReturnStrategy` attempted to land at the intermediate delivery pad instead of flying back to the home base.
- **Root Cause**: PX4 automatically updates its internal home position to the current GPS location whenever the vehicle is armed. When the vehicle re-armed on the landing pad after cargo delivery, PX4's new home position was overwritten with the pad's coordinates.
- **Solution**: Captured and permanently locked the initial home base coordinates in `FlightRuntimeNode` upon the **very first arming event** of the sortie. `ReturnStrategy` now explicitly uses this locked origin coordinate for RTL rather than querying PX4's dynamic home topic.
- **Architectural Lesson**: Never rely on external autopilot state for mission-critical reference coordinates across multi-leg missions. The companion application must own its mission origin reference.

---

### Bug 3: Multi-Sortie Ground Disarm vs Mid-Air Override Interlock
- **Commit**: `b5440e0` (*fix(flight): resolve ground disarm deactivation interlock and second takeoff re-arm climb*)
- **Symptom**: Whenever the drone landed on the ground and automatically disarmed, `FullSelfDrivingModeExecutor::onDeactivate` fired and erroneously flagged a `ManualTakeover` emergency state.
- **Root Cause**: `px4_ros2_cpp` invokes `onDeactivate` both when an operator manually takes over in flight *and* when the autopilot transitions to disarmed on the ground.
- **Solution**: Updated `MissionCoordinator::handle_takeover` to inspect the current strategy:
  ```cpp
  if (current_strategy_ == flight::StrategyType::LANDED_VERIFIED ||
      current_strategy_ == flight::StrategyType::PAYLOAD_OPERATION ||
      current_strategy_ == flight::StrategyType::TAKEOFF_AFTER_DELIVERY ||
      current_strategy_ == flight::StrategyType::RETURN_LANDED) {
    return; // Normal ground disarm — do NOT flag manual takeover
  }
  ```
- **Architectural Lesson**: Context matters in lifecycle event handling. An event (disarm) that is an anomaly in one phase (cruise) is expected behavior in another (landed).

---

### Bug 4: Auto-Takeoff Race Condition after Sortie Return
- **Commit**: `52e2d15` (*fix(runtime): prevent auto-takeoff race condition after return by requiring operator re-arm trigger*)
- **Symptom**: After completing a sortie and returning to home (`RETURN_LANDED`), the mode executor immediately attempted to re-arm and take off again without operator intervention.
- **Root Cause**: `FullSelfDrivingModeExecutor::onActivate` was called automatically by PX4 upon switching modes, and immediately triggered `trigger_takeoff_sequence()` if `isInCharge()` was true.
- **Solution**: Introduced a state gate `was_disarmed_after_return_` requiring an explicit operator re-arm command before a new takeoff sequence can begin.
- **Architectural Lesson**: Always enforce human-in-the-loop (HITL) authorization gates between sequential autonomous sorties.

---

### Bug 5: Hard Arming Rejection Gates (Target & Cargo Prerequisites)
- **Commit**: `95b3f76`, `401de8a`, `a71820a` (*fix(flight,payload): enforce hard arming rejection and multi-sortie payload readiness checks*)
- **Symptom**: An operator could arm the drone and take off before selecting a target landing pad or before securing cargo, causing the drone to enter `SEARCH` with no valid target identity.
- **Root Cause**: `FullSelfDrivingMode::checkArmingAndRunConditions` was only checking PX4 internal sensor health, not domain-level mission prerequisites.
- **Solution**: Connected `MissionContext` and `PayloadController` to the arming check reporter:
  - Reject arming if `target_identity` is not committed.
  - Reject arming on initial takeoff if `payload.is_secured == false`.
  - Allow post-delivery re-arming when `payload.cargo_loaded == false` (cargo released).
- **Architectural Lesson**: Safety arming checks must incorporate application-layer mission readiness, not just low-level IMU/GPS health.

---

### Bug 6: ESP32 Serial DTR/RTS Auto-Reset & Bootloader Drain
- **Commit**: `5dea0e8`, `031ff78` (*fix(serial): add DTR/RTS handling and bootloader log draining for ESP32 auto-reset*)
- **Symptom**: When `esp32_gripper_bridge.py` opened `/dev/ttyUSB0`, the first payload command failed with a timeout.
- **Root Cause**: Standard USB-to-UART bridge ICs (CP2102/CH340) toggle DTR and RTS low when opened, triggering the ESP32 hardware reset line. The microcontroller rebooted, printed ASCII bootloader headers at non-standard baud rates, and corrupted the incoming JSON command stream.
- **Solution**:
  1. Disabled DTR/RTS assertions on port open.
  2. Implemented a 1.2-second startup flush and drain loop to discard bootloader noise before entering the active command loop.
- **Architectural Lesson**: Hardware UARTs behave differently than software IPC. Embedded bridge drivers must account for microcontroller reset electrical characteristics.

---

### Bug 7: Dynamic TF2 Geodetic Projection from Live PX4 GPS
- **Commit**: `e2c214a`, `4136842` (*feat(registry): implement dynamic TF2 geodetic projection and live PX4 GPS auto-origin*)
- **Symptom**: Landing pad GPS coordinates recorded in `PadRegistry` had a constant offset when simulated in different Gazebo worlds.
- **Root Cause**: Early prototypes hardcoded the origin GPS coordinate of the KMITL airfield $(13.7313^\circ\text{N}, 100.7899^\circ\text{E})$ in the conversion math.
- **Solution**: Integrated live PX4 `HomePosition` and `VehicleGlobalPosition` uORB streams through `tf2_ros` and WGS84 geodesic transforms. The origin is now resolved dynamically at runtime.
- **Architectural Lesson**: Eliminate magic numbers and hardcoded geographic assumptions from perception and registry layers.

---

### Bug 8: Colcon Build Memory Limits & Parallel Worker OOM
- **Commit**: `b9dd0ec` (*fix(build): limit colcon parallel workers and memory usage to prevent OOM crash*)
- **Symptom**: `colcon build` crashed with `c++: fatal error: Killed signal terminated program cc1plus` during multi-core compilation in Docker.
- **Root Cause**: `gcc`/`clang` compiling heavy template headers (`Eigen`, `OpenCV`, `px4_ros2_cpp`, `yaml-cpp`) on all available CPU threads exceeded available container RAM (OOM killer).
- **Solution**: Configured build arguments:
  ```bash
  colcon build --symlink-install \
    --packages-select full_self_driving \
    --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=2 -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF
  ```
- **Architectural Lesson**: Build configurations must account for template-heavy C++ compilation memory footprints in containerized environments.

---

### Bug 9: Cross-Airfield Direct Navigation Timeout & Strategy Failure Handling
- **Commit**: `87a1d65` (*fix(flight): increase direct navigation timeout to 120s and add strategy failure handling*)
- **Symptom**: In the Survey-First flight mode (`complete_grid_first`), after surveying 100% of the grid, the drone began flying in `DIRECT` towards Target Pad 2, but suddenly stopped and hovered stationary in mid-air at 15m altitude, 46m away from the target pad.
- **Root Cause**:
  1. The distance from the final search waypoint across the airfield back to Pad 2 was ~150m. Flying at $5\text{ m/s}$ with turns took ~35s.
  2. `DirectStrategy` had a default `direct_timeout_s` of only 30.0s. At $t=30.019\text{s}$, it called `fail()` and stopped streaming setpoints to PX4.
  3. `FullSelfDrivingMode` had no `strategy_failed_cb_` registered, so when `DirectStrategy` failed, it never triggered a fallback and left the drone silently idling in the air.
- **Solution**:
  1. Increased `direct_timeout_s` from 30.0s to 120.0s in `DirectStrategy` and `MissionCoordinator`.
  2. Added virtual `is_failed()` and `failure_reason()` interface to `InternalStrategy`.
  3. Added `set_strategy_failed_callback` in `FullSelfDrivingMode` and wired `coordinator_->handle_direct_fallback()` in `FlightRuntimeNode` to safely recover from any flight strategy timeout.
- **Architectural Lesson**:
  1. Trajectory and waypoint timeouts must account for worst-case geographic transit distances across large operational areas.
  2. Strategy lifecycle handlers must handle failure paths with fallback recovery mechanisms, not just nominal completion paths.

---

### Bug 10: Precision Land Ground Disarm False Takeover Latch & Auto-Rearm Sequence
- **Commit**: `77f0b3a` (*fix(flight): resolve precision land ground takeover latch, auto rearm for second takeoff, and transit out altitude*)
- **Symptom**: When the drone touched down on the delivery target pad during `PRECISION_LAND`, the system became permanently idle in `HOLD` (phase 14) and refused to release cargo or take off for the second flight.
- **Root Cause**:
  1. Upon touchdown, PX4's autopilot land detector fired and automatically disarmed the vehicle on the ground.
  2. Ground disarming invoked `FullSelfDrivingModeExecutor::onDeactivate()`, which called `MissionCoordinator::handle_takeover()`.
  3. `handle_takeover()` did not exempt ground disarms during `PRECISION_LAND` or `RETURN_STRATEGY`, misinterpreting normal landing disarms as manual pilot takeovers (`takeover_active_ = true; current_strategy_ = HOLD`).
  4. With `takeover_active_` latched, `MissionCoordinator::request_transition()` rejected all autonomous transitions.
  5. Furthermore, upon payload release completion, PX4 required an explicit `arm()` and `trigger_takeoff_sequence()` command to relaunch the disarmed drone for the second takeoff (`TAKEOFF_AFTER_DELIVERY`).
- **Solution**:
  1. Filtered out ground disarms (`snapshot.is_landed == true`) during landing and payload strategies from triggering the manual takeover latch.
  2. Added periodic ground touchdown verification in `FlightRuntimeNode` to cleanly transition from `PRECISION_LAND` to `LANDED_VERIFIED` and `PAYLOAD_OPERATION`.
  3. Implemented automatic `executor_->arm()` and `executor_->trigger_takeoff_sequence()` execution upon successful payload delivery to launch the second flight.
- **Architectural Lesson**: Autopilot middleware lifecycles differ between airborne and ground states. Autonomy state machines must differentiate between flight-time pilot takeovers and physical ground landing disarms.

---

### Bug 11: Transit Out Corridor Altitude Parameter Mismatch
- **Commit**: `77f0b3a` (*fix(flight): resolve precision land ground takeover latch, auto rearm for second takeoff, and transit out altitude*)
- **Symptom**: During the second flight (`TRANSIT_OUT`), the drone flew the outbound corridor at 12.0m altitude instead of the configured 20.0m transit altitude.
- **Root Cause**: `MissionCoordinator::request_transition` bound `cfg.search_altitude_m` (12.0m) to `route.set_transit_altitude_above_home_m()` during `TAKEOFF_AFTER_DELIVERY -> TRANSIT_OUT` instead of `cfg.transit_altitude_m` (20.0m).
- **Solution**: Corrected the configuration binding across `TAKEOFF_AFTER_DELIVERY -> TRANSIT_OUT` and `TRANSIT_OUT -> RETURN_STRATEGY` to authoritative `cfg.transit_altitude_m`.
- **Architectural Lesson**: Parameter bindings in state machine transition matrices must strictly reflect distinct operational flight corridors (transit corridor vs survey grid).

