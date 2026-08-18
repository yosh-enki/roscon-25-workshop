# Task 12: Payload Delivery & Sortie Completion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the complete post-touchdown sortie sequence: disarmed/in-flight payload operation with safety gates and idempotency, second takeoff, TransitOut outbound route execution, ReturnStrategy recovery landing at base, auto-disarm, and sortie lifecycle completion, verified by comprehensive property and integration tests.

**Architecture:** 
- `PayloadController` & `SimulationPayloadAdapter`: Manages preflight preparation (`OPEN_FOR_LOADING`, `VERIFY_SECURED`, `PREPARE_FOR_SORTIE`) via disarmed gateway and in-flight release via internal strategy only. Enforces idempotency and non-retrying unknown results (Property 14).
- `PayloadOperationStrategy`: Executes gated payload delivery after confirmed touchdown (`LANDED_VERIFIED`), logging durable intent before actuation and durable result after feedback.
- `TransitOutStrategy` & Second Takeoff: Manages climb back to cruise altitude (15.0m AGL) and follows canonical outbound waypoints using `px4_ros2::GotoGlobalSetpointType` with course heading alignment.
- `ReturnStrategy`: Manages approach to home base, vertical descent, touchdown detection, auto-disarm, and transitions mission context to `COMPLETE`.
- Property Tests: Validates Property 14 (Payload Safety), Property 15 (Return Strategy Explicitness), and Property 11 (Mission Sequence Ordering).

**Tech Stack:** C++17, ROS 2 Humble (`rclcpp`, `rclcpp_lifecycle`), `px4_ros2_cpp`, Eigen3, GoogleTest, YAML-CPP, OpenSSL SHA-256.

---

## Global Constraints
- Target package: `full_self_driving`
- Zero dependencies on deprecated prototype packages (`transit_out`, `payload`, etc.)
- All flight phase transitions are strictly owned and decided by `MissionCoordinator` (Property 12)
- All gateway mutating operations remain strictly disarmed-only (Property 2)
- All durable state transitions must be journaled in `PersistenceManager` (Property 16)
- In-flight payload operation occurs only after `LANDED_VERIFIED` stability dwell and passes all safety gates (Property 14)
- Outbound and recovery behavior comes strictly from configured route without automatic inbound reversal (Property 15)

---

### Task 1: Payload Controller & Simulation Payload Adapter (Task 12.1)

**Files:**
- Create: `full_self_driving/src/payload/payload_adapter.hpp`
- Create: `full_self_driving/src/payload/simulation_payload_adapter.hpp`
- Create: `full_self_driving/src/payload/simulation_payload_adapter.cpp`
- Create: `full_self_driving/src/payload/payload_controller.hpp`
- Create: `full_self_driving/src/payload/payload_controller.cpp`
- Modify: `full_self_driving/CMakeLists.txt`
- Modify: `full_self_driving/src/gateway/fsd_gateway.hpp` / `fsd_gateway.cpp`
- Modify: `full_self_driving/src/gateway/fsd_gateway_node.hpp` / `fsd_gateway_node.cpp`
- Test: `full_self_driving/test/payload/payload_controller_test.cpp`

**Interfaces:**
- Produces: `fsd_payload_core` library with `PayloadController`, `PayloadAdapter`, `SimulationPayloadAdapter`, and preflight gateway integration via `srv/PreparePayload.srv`.

- [ ] **Step 1: Write unit test for PayloadController and SimulationPayloadAdapter**
- [ ] **Step 2: Create PayloadAdapter interface and SimulationPayloadAdapter**
- [ ] **Step 3: Implement PayloadController with preflight operations and state tracking**
- [ ] **Step 4: Integrate PayloadController into FsdGateway and FlightRuntimeNode**
- [ ] **Step 5: Run tests and verify preflight preparation pass**

---

### Task 2: Durable Internal Payload Operation Strategy (Task 12.2)

**Files:**
- Create: `full_self_driving/src/flight/strategies/payload_operation_strategy.hpp`
- Create: `full_self_driving/src/flight/strategies/payload_operation_strategy.cpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.hpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.cpp`
- Modify: `full_self_driving/src/flight/full_self_driving_mode.cpp`
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp`
- Modify: `full_self_driving/CMakeLists.txt`
- Test: `full_self_driving/test/flight/payload_operation_test.cpp`

**Interfaces:**
- Consumes: `PayloadController`, `MissionCoordinator`, `PersistenceManager`
- Produces: `PayloadOperationStrategy` handling `LANDED_VERIFIED -> PAYLOAD_OPERATION -> TAKEOFF_AFTER_DELIVERY` transitions with durable intent and result journaling.

- [ ] **Step 1: Write failing test for payload operation strategy and coordinator transitions**
- [ ] **Step 2: Implement PayloadOperationStrategy with timeout, idempotency, and durable intent/result**
- [ ] **Step 3: Wire PayloadOperationStrategy into MissionCoordinator and FullSelfDrivingMode**
- [ ] **Step 4: Wire strategy completion in FlightRuntimeNode**
- [ ] **Step 5: Run tests to verify payload operation pass**

---

### Task 3: Second Takeoff & TransitOut Flight Strategy (Task 12.3)

**Files:**
- Create: `full_self_driving/src/flight/strategies/transit_out_strategy.hpp`
- Create: `full_self_driving/src/flight/strategies/transit_out_strategy.cpp`
- Create: `full_self_driving/test/fixtures/prototype_behavior/transit_out/golden_transit_out_waypoints.yaml`
- Create: `full_self_driving/test/fixtures/prototype_behavior/transit_out/golden_transit_out_trace.yaml`
- Modify: `full_self_driving/src/domain/mission_coordinator.hpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.cpp`
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp`
- Modify: `full_self_driving/CMakeLists.txt`
- Test: `full_self_driving/test/flight/transit_out_parity_test.cpp`

**Interfaces:**
- Consumes: `px4_ros2::GotoGlobalSetpointType`, `Px4StateCache`, `domain::Route`
- Produces: `TransitOutStrategy` executing canonical outbound waypoints and waypoint checkpointing.

- [ ] **Step 1: Add golden fixtures for TransitOut waypoints and trace**
- [ ] **Step 2: Write failing TransitOut parity test**
- [ ] **Step 3: Implement TransitOutStrategy with global goto setpoints and course heading**
- [ ] **Step 4: Update MissionCoordinator for TAKEOFF_AFTER_DELIVERY -> TRANSIT_OUT**
- [ ] **Step 5: Run parity tests and verify 100% pass**

---

### Task 4: ReturnStrategy, Recovery Landing & Sortie Completion (Task 12.4)

**Files:**
- Create: `full_self_driving/src/flight/strategies/return_strategy.hpp`
- Create: `full_self_driving/src/flight/strategies/return_strategy.cpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.hpp`
- Modify: `full_self_driving/src/domain/mission_coordinator.cpp`
- Modify: `full_self_driving/src/runtime/flight_runtime_node.cpp`
- Modify: `full_self_driving/CMakeLists.txt`
- Test: `full_self_driving/test/flight/return_strategy_test.cpp`

**Interfaces:**
- Consumes: `MissionCoordinator`, `MissionContext`, `Px4StateCache`
- Produces: `ReturnStrategy` completing approach to home base, vertical touchdown detection, auto-disarm, and sortie finalization to `ConfigState::COMPLETE`.

- [ ] **Step 1: Write failing test for ReturnStrategy and completion transitions**
- [ ] **Step 2: Implement ReturnStrategy with home approach, touchdown verify, and auto-disarm**
- [ ] **Step 3: Wire ReturnStrategy completion to MissionContext::COMPLETE**
- [ ] **Step 4: Run tests and verify return & completion pass**

---

### Task 5: Property 14 Test Suite (Payload Operation Safety - Task 12.5)

**Files:**
- Create: `full_self_driving/test/property/property_14_payload_safety.cpp`
- Modify: `full_self_driving/CMakeLists.txt`

**Interfaces:**
- Tests Design Property 14: Gated payload operation, idempotency, non-retrying unknown results, and durable boundaries.

- [ ] **Step 1: Implement Property 14 test suite**
- [ ] **Step 2: Register fsd_property_14_payload_safety in CMakeLists.txt**
- [ ] **Step 3: Run and verify all Property 14 tests pass**

---

### Task 6: Property 15 Test Suite (Return Strategy Explicitness - Task 12.6)

**Files:**
- Create: `full_self_driving/test/property/property_15_return_strategy.cpp`
- Modify: `full_self_driving/CMakeLists.txt`

**Interfaces:**
- Tests Design Property 15: Explicit outbound and return behavior, validation of return routes, rejection of implicit reverse route assumptions.

- [ ] **Step 1: Implement Property 15 test suite**
- [ ] **Step 2: Register fsd_property_15_return_strategy in CMakeLists.txt**
- [ ] **Step 3: Run and verify all Property 15 tests pass**

---

### Task 7: Property 11 & End-to-End Sortie Integration Test (Task 12.7)

**Files:**
- Create: `full_self_driving/test/property/property_11_mission_sequence.cpp`
- Create: `full_self_driving/test/integration/nominal_sortie_sequence_test.cpp`
- Modify: `full_self_driving/CMakeLists.txt`

**Interfaces:**
- Tests Design Property 11 & Full Sortie: Preflight -> Takeoff -> TransitIn -> Direct/Search -> PrecisionLand -> Touchdown Verify -> Payload Release -> Second Takeoff -> TransitOut -> Return -> Touchdown -> Disarm -> Complete.

- [ ] **Step 1: Implement Property 11 test suite**
- [ ] **Step 2: Implement nominal sortie integration test**
- [ ] **Step 3: Register test targets in CMakeLists.txt**
- [ ] **Step 4: Run full test suite and confirm 100% pass across all tests**

---

### Task 8: Documentation & Manual Update

**Files:**
- Modify: `full_self_driving/MANUAL.md`

- [ ] **Step 1: Document Section 12 (Payload Delivery & Sortie Completion) in MANUAL.md**
- [ ] **Step 2: Update verification commands, architecture diagrams, and test tallies**
