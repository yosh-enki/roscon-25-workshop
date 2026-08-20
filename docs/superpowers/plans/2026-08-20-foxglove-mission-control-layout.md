# Foxglove Mission Control Native Layout Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a comprehensive, production-ready Foxglove Studio native layout (`foxglove/roscon-25-workshop.json`) for full mission control of the autonomous drone stack, featuring service call panels, live vision, 3D tracking, and telemetry monitoring across Docker SITL and Host workstation boundaries.

**Architecture:** A single-screen 3-column Foxglove Studio layout (Command & Safety, Vision & 3D Spatial, Flight Telemetry & Pad Database) communicating with the containerized ROS 2 FSD stack via `foxglove_bridge` (`ws://localhost:8765`).

**Tech Stack:** Foxglove Studio JSON Layout Schema v1, ROS 2 Humble, `foxglove_bridge`, Python 3 validation scripting.

## Global Constraints
- Target Layout File: `foxglove/roscon-25-workshop.json`
- Service Targets: `/full_self_driving/select_target`, `/full_self_driving/prepare_payload`, `/full_self_driving/emergency_stop`
- Topic Targets: `/full_self_driving/state`, `/full_self_driving/readiness`, `/full_self_driving/perception/annotated_image`, `/full_self_driving/perception/live_target_lock`, `/full_self_driving/payload/status`, `/full_self_driving/telemetry`, `/full_self_driving/pad_registry`
- Deployment Model: SITL on Docker (port 8765 exposed), Foxglove Studio on Host machine.

---

### Task 1: Foxglove Layout Validator Script

**Files:**
- Create: `scripts/validate_foxglove_layout.py`
- Test: `scripts/validate_foxglove_layout.py`

**Interfaces:**
- Consumes: `foxglove/roscon-25-workshop.json`
- Produces: Command-line validator checking valid JSON, panel configById mapping, layout split-tree consistency, and required topic/service references.

- [ ] **Step 1: Write validation script and test structure**

```python
#!/usr/bin/env python3
"""Validation script for Foxglove Studio layout configuration."""

import json
import sys
from pathlib import Path

REQUIRED_SERVICES = [
    "/full_self_driving/select_target",
    "/full_self_driving/prepare_payload",
    "/full_self_driving/emergency_stop",
]

REQUIRED_TOPICS = [
    "/full_self_driving/perception/annotated_image",
    "/full_self_driving/state",
    "/full_self_driving/readiness",
    "/full_self_driving/perception/live_target_lock",
    "/full_self_driving/telemetry",
    "/full_self_driving/pad_registry",
]

def validate_layout(layout_path: Path) -> bool:
    if not layout_path.exists():
        print(f"Error: {layout_path} does not exist", file=sys.stderr)
        return False

    try:
        with open(layout_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error parsing JSON: {e}", file=sys.stderr)
        return False

    if "configById" not in data or "layout" not in data:
        print("Error: Missing 'configById' or 'layout' keys", file=sys.stderr)
        return False

    config_keys = set(data["configById"].keys())

    # Traverse layout tree to ensure all panel references exist in configById
    def check_layout_node(node):
        if isinstance(node, str):
            if node not in config_keys:
                print(f"Error: Layout references unknown panel '{node}'", file=sys.stderr)
                return False
            return True
        elif isinstance(node, dict):
            first = node.get("first")
            second = node.get("second")
            return check_layout_node(first) and check_layout_node(second)
        return False

    if not check_layout_node(data["layout"]):
        return False

    # Check serialized contents for required topics and services
    raw_str = json.dumps(data)
    missing_services = [s for s in REQUIRED_SERVICES if s not in raw_str]
    missing_topics = [t for t in REQUIRED_TOPICS if t not in raw_str]

    if missing_services:
        print(f"Error: Missing required services: {missing_services}", file=sys.stderr)
        return False

    if missing_topics:
        print(f"Error: Missing required topics: {missing_topics}", file=sys.stderr)
        return False

    print("Foxglove layout validated successfully.")
    return True

if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("foxglove/roscon-25-workshop.json")
    success = validate_layout(path)
    sys.exit(0 if success else 1)
```

- [ ] **Step 2: Run test to verify it fails on old layout**

Run: `python3 scripts/validate_foxglove_layout.py foxglove/roscon-25-workshop.json`
Expected: FAIL with "Missing required services"

- [ ] **Step 3: Make script executable**

Run: `chmod +x scripts/validate_foxglove_layout.py`

- [ ] **Step 4: Commit validator**

```bash
git add scripts/validate_foxglove_layout.py
git commit -m "test: add automated Foxglove layout validation script"
```

---

### Task 2: Implement Production Foxglove Mission Control Layout

**Files:**
- Modify: `foxglove/roscon-25-workshop.json`

**Interfaces:**
- Consumes: FSD ROS 2 topic schemas and service schemas.
- Produces: Fully configured 3-column Foxglove Studio layout JSON containing:
  - `CallService!select_target`: prefilled for `/full_self_driving/select_target`
  - `CallService!prepare_payload`: prefilled for `/full_self_driving/prepare_payload`
  - `CallService!emergency_stop`: prefilled for `/full_self_driving/emergency_stop`
  - `RawMessages!readiness`: for `/full_self_driving/readiness`
  - `ImageView!fsd_annotated`: for `/full_self_driving/perception/annotated_image`
  - `3D!vehicle_scene`: 3D scene with URDF and TF tracking
  - `RawMessages!fsd_state`: for `/full_self_driving/state`
  - `RawMessages!live_lock`: for `/full_self_driving/perception/live_target_lock`
  - `Plot!altitude`: for `/full_self_driving/telemetry.altitude_m`
  - `Plot!battery_speed`: for `/full_self_driving/telemetry.battery_percentage` & `ground_speed_m_s`
  - `RawMessages!pad_registry`: for `/full_self_driving/pad_registry`

- [ ] **Step 1: Generate updated `foxglove/roscon-25-workshop.json` with 3-column split tree**
- [ ] **Step 2: Run validator script to verify layout passes**

Run: `python3 scripts/validate_foxglove_layout.py foxglove/roscon-25-workshop.json`
Expected: PASS with "Foxglove layout validated successfully."

- [ ] **Step 3: Commit updated layout**

```bash
git add foxglove/roscon-25-workshop.json
git commit -m "feat(foxglove): implement 3-column mission control native layout"
```

---

### Task 3: Update Operational Documentation in MANUAL.md & README.md

**Files:**
- Modify: `full_self_driving/MANUAL.md:1800-1970`
- Modify: `README.md`

**Interfaces:**
- Consumes: Spec requirements and Docker/Host setup instructions.
- Produces: Clear, illustrated documentation for Host workstation Foxglove connection, layout import instructions, pre-flight workflow, and emergency stop procedures.

- [ ] **Step 1: Update Section 18 in `full_self_driving/MANUAL.md` with Foxglove Studio usage details**
- [ ] **Step 2: Update `README.md` Quickstart with Foxglove layout import guide**
- [ ] **Step 3: Commit documentation updates**

```bash
git add full_self_driving/MANUAL.md README.md
git commit -m "docs: add Foxglove Studio layout operational guide"
```

---

### Task 4: End-to-End Layout & Package Verification

**Files:**
- Test: `scripts/validate_foxglove_layout.py`
- Test: `full_self_driving` test suite

- [ ] **Step 1: Run automated Foxglove layout validation**

Run: `python3 scripts/validate_foxglove_layout.py`
Expected: Return code 0.

- [ ] **Step 2: Run existing package unit and property tests**

Run: `colcon test --packages-select full_self_driving && colcon test-result --verbose`
Expected: All tests pass.

- [ ] **Step 3: Final verification commit & summary**
