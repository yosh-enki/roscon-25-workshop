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
