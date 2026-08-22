# Survey-First Flight Plan Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the `COMPLETE_FULL_GRID_FIRST` search policy in `full_self_driving`, enabling full-grid survey mapping on the first sortie followed by direct navigation to the target pad, and pure direct flights on subsequent sorties.

**Architecture:** Extend `EngineeringConfig`/`fsd_parameters.yaml` with `search_policy`, adapt `MissionCoordinator` target lock handling and search completion callbacks to transition `SEARCH` $\rightarrow$ `DIRECT` $\rightarrow$ `PRECISION_LAND`, and update `FlightRuntimeNode` and unit tests.

**Tech Stack:** ROS 2 (Humble/Iron/Jazzy), C++17, Google Test, `px4_ros2_cpp`.

## Global Constraints
- All changes must be made on branch `feat/improve-flight-plan`.
- Backward compatibility with `interrupt_on_target` must be preserved.
- All tests in `full_self_driving` must compile and pass green with zero regressions.

---

### Task 1: Domain Configuration & Parameter Extension
**Files:**
- Modify: `full_self_driving/src/domain/engineering_config.hpp`
- Modify: `full_self_driving/src/domain/engineering_config.cpp`
- Modify: `full_self_driving/config/fsd_parameters.yaml`
- Modify: `full_self_driving/config/schemas/engineering_config.schema.json`
- Test: `full_self_driving/test/domain/engineering_config_test.cpp` (if exists) or colcon build

- [ ] **Step 1:** Add `search_policy` (default: `"complete_grid_first"`) to `RoutePolicy` in `engineering_config.hpp`.
- [ ] **Step 2:** Update YAML parsing and serialization in `engineering_config.cpp`.
- [ ] **Step 3:** Update `config/fsd_parameters.yaml` with `routes.search_policy: "complete_grid_first"`.
- [ ] **Step 4:** Update `engineering_config.schema.json`.

---

### Task 2: Mission Coordinator Survey Policy & Transition Logic
**Files:**
- Modify: `full_self_driving/src/domain/mission_coordinator.hpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.cpp`

- [ ] **Step 1:** In `mission_coordinator.hpp`, add `search_policy_` field, getters/setters, and declaration for `bool handle_search_completed()`.
- [ ] **Step 2:** In `handle_target_lock_update()`, if `search_policy_ == "complete_grid_first"`, suppress immediate preemption to `PRECISION_LAND`.
- [ ] **Step 3:** Implement `handle_search_completed()`:
  - Lookup target in `PadRegistry`.
  - If found: Transition `SEARCH` $\rightarrow$ `DIRECT` with the target pad coordinates.
  - If not found: Transition `SEARCH` $\rightarrow$ `RETURN_STRATEGY` (or `HOLD`).
- [ ] **Step 4:** In `request_transition(flight::StrategyType::DIRECT)`, ensure transition from `SEARCH` $\rightarrow$ `DIRECT` is permitted and properly instantiates `DirectStrategy`.

---

### Task 3: Flight Runtime Node Completion Handler Integration
**Files:**
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp`

- [ ] **Step 1:** In `FlightRuntimeNode::initialize_components()`, sync `search_policy` from `config_` to `coordinator_`.
- [ ] **Step 2:** In `mode_->set_strategy_completed_callback`:
  - When `completed_type == flight::StrategyType::SEARCH`, invoke `coordinator_->handle_search_completed()`.

---

### Task 4: Unit Testing & Verification (TDD)
**Files:**
- Modify: `full_self_driving/test/domain/mission_coordinator_test.cpp` (or add new test case in `test/flight/`)
- Test: Build and run test suites with `colcon test --packages-select full_self_driving`.

- [ ] **Step 1:** Write unit test verifying that with `complete_grid_first`, target locks do not interrupt `SEARCH`.
- [ ] **Step 2:** Write unit test verifying that `handle_search_completed()` transitions to `DIRECT` when target is in `PadRegistry`.
- [ ] **Step 3:** Run full build and test suite to confirm zero regressions.
