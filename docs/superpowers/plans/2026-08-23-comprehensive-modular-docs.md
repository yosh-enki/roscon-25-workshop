# Comprehensive Modular Documentation Suite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a comprehensive, production-grade, 9-module documentation suite inside `full_self_driving/docs/` reflecting all system features, architectural designs, and the full development/bug history from git.

**Architecture:** A modular Diátaxis documentation structure under `full_self_driving/docs/` detailing Architecture, Flight Strategies, Perception, Payload & HITL, Foxglove Mission Control, Configuration & Security, Git Development Journey & Bug Post-Mortems, and Operations Runbook.

**Tech Stack:** ROS 2 (Humble/Iron/Jazzy), C++17, PX4 Autopilot (`px4_ros2_cpp`), Gazebo Harmonic, Foxglove Studio, OpenCV ArUco, ESP32 Arduino C++, Markdown (GitHub-flavored with Mermaid diagrams & file links).

## Global Constraints
- All documents must be placed under `/home/yosh/roscon-25-workshop/full_self_driving/docs/`.
- All code and file links must use GitHub-style markdown links with `file:///` URI scheme pointing to exact absolute paths.
- Accurate topic names, QoS profiles, message schemas, and parameter keys must match `fsd_parameters.yaml` and code.
- Post-mortems in Module 7 must match the exact Git history and commit hashes.

---

### Task 1: Module 0 - Table of Contents & Navigation Map (`README.md`)
**Files:**
- Create: `full_self_driving/docs/README.md`
- [ ] **Step 1:** Write `full_self_driving/docs/README.md` with system overview, architecture diagram, module index, and navigation links.
- [ ] **Step 2:** Verify all links and formatting.

---

### Task 2: Module 1 - Architecture & Design Principles (`01_architecture_and_design_principles.md`)
**Files:**
- Create: `full_self_driving/docs/01_architecture_and_design_principles.md`
- [ ] **Step 1:** Write `01_architecture_and_design_principles.md` covering layered architecture, SOLID principles in ROS 2, `px4_ros2_cpp` Registered Mode authority (vs legacy offboard), and Hardware Deferral Gate.
- [ ] **Step 2:** Verify file references and diagrams.

---

### Task 3: Module 2 - Flight Runtime & Strategy Engine (`02_flight_runtime_and_strategies.md`)
**Files:**
- Create: `full_self_driving/docs/02_flight_runtime_and_strategies.md`
- [ ] **Step 1:** Write `02_flight_runtime_and_strategies.md` detailing the Mission Coordinator FSM, all 8 flight strategies, trajectory control, takeover latching, and emergency failsafe.
- [ ] **Step 2:** Verify state transition diagrams and parameter names.

---

### Task 4: Module 3 - Perception & Scoped Pad Registry (`03_perception_and_pad_registry.md`)
**Files:**
- Create: `full_self_driving/docs/03_perception_and_pad_registry.md`
- [ ] **Step 1:** Write `03_perception_and_pad_registry.md` covering `fsd_perception` Lifecycle Node, OpenCV `solvePnP`, SHA-256 calibration hashing, covariance matrix, TargetCoordinator, and Scoped Pad Registry with dynamic TF2 geodesy.
- [ ] **Step 2:** Verify mathematical formulations and topic QoS.

---

### Task 5: Module 4 - Payload Subsystems & HITL Testbed (`04_payload_and_hitl_testbed.md`)
**Files:**
- Create: `full_self_driving/docs/04_payload_and_hitl_testbed.md`
- [ ] **Step 1:** Write `04_payload_and_hitl_testbed.md` covering Payload HAL (`SimulationPayloadAdapter`, `Px4GripperPayloadAdapter`, `HardwarePayloadAdapter`), ESP32 Arduino firmware, serial JSON protocol, DTR/RTS auto-reset draining, and wiring schematics.
- [ ] **Step 2:** Verify serial protocol commands and GPIO pinouts.

---

### Task 6: Module 5 - Foxglove Mission Control (`05_foxglove_mission_control.md`)
**Files:**
- Create: `full_self_driving/docs/05_foxglove_mission_control.md`
- [ ] **Step 1:** Write `05_foxglove_mission_control.md` detailing the 3-column Aerospace HUD layout, custom extension (`.foxe`), target selection chips, cargo controls, Plan Manager UI, and WebSocket bridge setup.
- [ ] **Step 2:** Verify layout panel IDs and topic bindings.

---

### Task 7: Module 6 - Configuration & Security Hardening (`06_configuration_and_security.md`)
**Files:**
- Create: `full_self_driving/docs/06_configuration_and_security.md`
- [ ] **Step 1:** Write `06_configuration_and_security.md` with complete `fsd_parameters.yaml` parameter catalog, JSON schemas, SROS2 keystore PKI, DDS security profiles, and Gateway negative security boundaries.
- [ ] **Step 2:** Verify parameter types, defaults, and security governance rules.

---

### Task 8: Module 7 - Development Journey & Lessons Learned (`07_development_journey_and_lessons_learned.md`)
**Files:**
- Create: `full_self_driving/docs/07_development_journey_and_lessons_learned.md`
- [ ] **Step 1:** Write `07_development_journey_and_lessons_learned.md` analyzing the complete development journey from git log (Tasks 1-16 + HITL), including 10+ critical bug post-mortems (root causes, code diffs, architectural lessons).
- [ ] **Step 2:** Verify commit hashes and technical details against git history.

---

### Task 9: Module 8 - Operations & Troubleshooting Runbook (`08_operations_and_troubleshooting_runbook.md`)
**Files:**
- Create: `full_self_driving/docs/08_operations_and_troubleshooting_runbook.md`
- [ ] **Step 1:** Write `08_operations_and_troubleshooting_runbook.md` detailing single-launch simulation, HITL runner with GPU acceleration, headless CI testing, verification checklists, and troubleshooting matrix.
- [ ] **Step 2:** Verify all shell commands and diagnostic procedures.

---

### Task 10: Suite Integration & Link Validation
**Files:**
- Modify: `full_self_driving/docs/README.md` (if needed)
- [ ] **Step 1:** Verify all internal markdown links and cross-references across all 9 documents.
- [ ] **Step 2:** Confirm completeness and accuracy against the entire codebase.
