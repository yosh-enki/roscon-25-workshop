# Search Missing Target Egress Transit-Out & Sortie Target Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enforce egress transit corridor (`TRANSIT_OUT`) when search finishes without finding the target, and clear target selection upon sortie touchdown (`RETURN_LANDED`) to prevent flight loops.

**Architecture:** Update `MissionCoordinator::handle_search_completed()` to branch to `TRANSIT_OUT` when target is missing, add `clear_target()` to `MissionContext`, trigger target reset on `RETURN_LANDED` in `FlightRuntimeNode`, and update/add unit tests.

**Tech Stack:** ROS 2 (Humble), C++17, GoogleTest, `px4_ros2_cpp`.

**Spec:** `docs/superpowers/specs/2026-08-26-search-missing-target-transit-out-design.md`

## Global Constraints
- Target package: `full_self_driving`
- C++ standard: C++17
- All existing tests in `full_self_driving` must pass 100%

---

### Task 1: Add `clear_target()` Method to `MissionContext` and Unit Tests

**Files:**
- Modify: `full_self_driving/src/domain/mission_context.hpp`
- Modify: `full_self_driving/src/domain/mission_context.cpp`
- Modify: `full_self_driving/test/domain/mission_context_test.cpp`

**Interfaces:**
- Produces: `void MissionContext::clear_target()`

- [ ] **Step 1: Write failing test in `mission_context_test.cpp`**

```cpp
TEST_F(MissionContextTest, ClearTargetResetsTargetAndFailsReadiness)
{
  auto config = std::make_shared<EngineeringConfig>(
    EngineeringConfig::create_default_simulation_config());
  ctx_->set_engineering_config(config);

  std::string err;
  ASSERT_TRUE(ctx_->select_map_scenario("kmitl_airfield", "default_scenario", 0, &err));
  ASSERT_TRUE(ctx_->select_target(TargetIdentity(1, "DICT_4X4_50", "aavc2026"), 1, &err));
  auto vreport = ctx_->validate_selection(2);
  ASSERT_TRUE(vreport.is_valid);
  ASSERT_TRUE(ctx_->commit(vreport.token, 2, &err));

  std::vector<std::string> missing;
  EXPECT_TRUE(ctx_->check_readiness(true, true, true, &missing));

  // Now clear target
  ctx_->clear_target();
  EXPECT_FALSE(ctx_->get_selection().target.has_value());

  // Readiness must fail with MISSING_TARGET_IDENTITY
  missing.clear();
  EXPECT_FALSE(ctx_->check_readiness(true, true, true, &missing));
  bool found_missing_target = false;
  for (const auto & m : missing) {
    if (m.find("MISSING_TARGET_IDENTITY") != std::string::npos) {
      found_missing_target = true;
      break;
    }
  }
  EXPECT_TRUE(found_missing_target);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon test --packages-select full_self_driving --ctest-args -R mission_context_test`
Expected: Compilation failure or FAIL (method `clear_target` does not exist).

- [ ] **Step 3: Implement `clear_target()` in `MissionContext`**

In `full_self_driving/src/domain/mission_context.hpp`:
```cpp
void clear_target();
```

In `full_self_driving/src/domain/mission_context.cpp`:
```cpp
void MissionContext::clear_target()
{
  selection_.target.reset();
  selection_.selection_revision++;
  if (state_ == ConfigState::COMMITTED || state_ == ConfigState::VALIDATING) {
    state_ = ConfigState::CONFIGURING;
  }
  validation_token_.clear();
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon test --packages-select full_self_driving --ctest-args -R mission_context_test`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add full_self_driving/src/domain/mission_context.hpp full_self_driving/src/domain/mission_context.cpp full_self_driving/test/domain/mission_context_test.cpp
git commit -m "feat(domain): add clear_target to MissionContext and test"
```

---

### Task 2: Transition to `TRANSIT_OUT` when Target is Missing in `MissionCoordinator`

**Files:**
- Modify: `full_self_driving/src/domain/mission_coordinator.cpp:592-626`
- Modify: `full_self_driving/test/flight/acquisition_branch_test.cpp:402-421`

**Interfaces:**
- Consumes: `MissionCoordinator::handle_search_completed()`
- Produces: State transition `SEARCH -> TRANSIT_OUT` on missing target

- [ ] **Step 1: Update Test 14 in `test/flight/acquisition_branch_test.cpp` to expect `TRANSIT_OUT`**

```cpp
// 14. CompleteGridFirst: Search completion transitions to TRANSIT_OUT if target was NOT seen
TEST_F(AcquisitionBranchTest, Branch_CompleteGridFirst_SearchCompleteMissingTargetTransitionsToTransitOut)
{
  coordinator_->set_search_policy("complete_grid_first");
  auto wp = create_valid_working_plan();
  coordinator_->set_custom_search_plan(wp);

  EXPECT_TRUE(coordinator_->request_transition(flight::StrategyType::SEARCH));
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::SEARCH);

  // PadRegistry is empty (target was never seen)
  coordinator_->set_current_monotonic_ns(1000000000ULL + 1000000000ULL);
  coordinator_->set_battery_percentage(85.0);

  // Handle search completed -> Should transition to TRANSIT_OUT (Egress corridor)
  EXPECT_TRUE(coordinator_->handle_search_completed());
  EXPECT_EQ(coordinator_->get_current_strategy(), flight::StrategyType::TRANSIT_OUT);
  EXPECT_EQ(mode_->get_current_strategy_type(), flight::StrategyType::TRANSIT_OUT);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon test --packages-select full_self_driving --ctest-args -R acquisition_branch_test`
Expected: FAIL (received `RETURN_STRATEGY` instead of `TRANSIT_OUT`).

- [ ] **Step 3: Update `handle_search_completed()` in `mission_coordinator.cpp`**

In `full_self_driving/src/domain/mission_coordinator.cpp`:
```cpp
bool MissionCoordinator::handle_search_completed()
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (current_strategy_ != flight::StrategyType::SEARCH) {
    return false;
  }

  transition_trace_.push_back("SEARCH_GRID_COMPLETED");

  std::string rejection_reason;
  double direct_lat = 0.0, direct_lon = 0.0, direct_alt = 15.0;
  bool direct_ok = is_direct_eligible(&rejection_reason, &direct_lat, &direct_lon, &direct_alt);

  if (direct_ok) {
    transition_trace_.push_back("FLY-004 / EVT_SEARCH_TO_DIRECT_TRANSITION (lat=" +
      std::to_string(direct_lat) + ", lon=" + std::to_string(direct_lon) + ")");
    current_strategy_ = flight::StrategyType::DIRECT;
    instantiate_direct_strategy(direct_lat, direct_lon, direct_alt);
    return true;
  } else {
    transition_trace_.push_back("SURVEY_COMPLETE_TARGET_MISSING: " + rejection_reason + " -> TRANSIT_OUT");
    current_strategy_ = flight::StrategyType::TRANSIT_OUT;
    if (mode_) {
      Route route;
      if (has_custom_transit_out_route_) {
        route = custom_transit_out_route_;
      } else {
        route = Route::create_default_kmitl_transit_out_route();
        if (context_ && context_->get_resolved_config()) {
          const auto & cfg = context_->get_resolved_config()->routes;
          route.set_max_horizontal_speed_m_s(static_cast<float>(cfg.transit_out_speed_m_s));
          route.set_transit_altitude_above_home_m(cfg.transit_altitude_m);
          route.set_acceptance_radius_m(static_cast<float>(cfg.acceptance_radius_m));
          route.set_max_yaw_rate_deg_s(static_cast<float>(cfg.max_yaw_rate_deg_s));
        }
      }
      mode_->set_strategy(std::make_unique<flight::TransitOutStrategy>(
        mode_->node(), mode_->goto_global_setpoint(), mode_->state_cache(), route, persistence_));
    }
    return true;
  }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `colcon test --packages-select full_self_driving --ctest-args -R acquisition_branch_test`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add full_self_driving/src/domain/mission_coordinator.cpp full_self_driving/test/flight/acquisition_branch_test.cpp
git commit -m "fix(flight): route through TRANSIT_OUT when search finishes without target"
```

---

### Task 3: Trigger `clear_target()` upon `RETURN_LANDED` in `FlightRuntimeNode`

**Files:**
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp:750-760, 955-968`

**Interfaces:**
- Consumes: `MissionContext::clear_target()`
- Produces: Cleared target upon disarmed touchdown at home base

- [ ] **Step 1: Add target clear to `FlightRuntimeNode` return strategy completion**

In `full_self_driving/src/runtime/flight_runtime_node.cpp`:
In the `RETURN_STRATEGY` completion callback:
```cpp
      } else if (completed_type == flight::StrategyType::RETURN_STRATEGY) {
        RCLCPP_INFO(get_logger(), "[RUNTIME] ReturnStrategy completed at Home Base. Transitioning to RETURN_LANDED and disarming...");
        was_disarmed_after_return_ = false;
        if (coordinator_) {
          coordinator_->request_transition(flight::StrategyType::RETURN_LANDED);
        }
        if (context_) {
          context_->clear_target();
          RCLCPP_INFO(get_logger(), "[RUNTIME] Sortie completed at Home Base. Target identity cleared to prevent loop.");
        }
        if (executor_) {
          executor_->disarm([](px4_ros2::Result result) {
            RCLCPP_INFO(rclcpp::get_logger("FlightRuntimeNode"),
              "[RUNTIME] Final mission disarm completed with result: %s",
              px4_ros2::resultToString(result));
          });
        }
      }
```

And in the periodic touchdown check for `RETURN_STRATEGY`:
```cpp
    if (current_strat == flight::StrategyType::RETURN_STRATEGY) {
      if (state_cache_) {
        auto snapshot = state_cache_->capture_snapshot();
        float vz = snapshot.local_velocity_ned.z();
        if (snapshot.is_landed || (snapshot.local_position_ned.z() >= -0.3f && std::abs(vz) < 0.25f)) {
          RCLCPP_INFO(get_logger(),
            "[RUNTIME] (Periodic) Return strategy touchdown verified at Home Base. Transitioning to RETURN_LANDED...");
          was_disarmed_after_return_ = false;
          coordinator_->request_transition(flight::StrategyType::RETURN_LANDED);
          if (context_) {
            context_->clear_target();
            RCLCPP_INFO(get_logger(), "[RUNTIME] (Periodic) Target identity cleared on return touchdown.");
          }
        }
      }
    }
```

- [ ] **Step 2: Run full package tests to verify no regressions**

Run: `colcon test --packages-select full_self_driving`
Expected: 100% PASS

- [ ] **Step 3: Commit**

```bash
git add full_self_driving/src/runtime/flight_runtime_node.cpp
git commit -m "fix(runtime): clear target identity upon return landed to prevent loop"
```

---

### Task 4: Full Test Suite and Verification

**Files:**
- Test: All tests in `full_self_driving`

- [ ] **Step 1: Run complete test suite**

Run:
```bash
colcon build --packages-select full_self_driving
colcon test --packages-select full_self_driving
colcon test-result --verbose
```
Expected: All tests pass with 0 failures.

- [ ] **Step 2: Final commit and cleanup**

```bash
git status
```
