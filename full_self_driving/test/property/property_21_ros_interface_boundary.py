import os
import re
import pytest

PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
MSG_DIR = os.path.join(PACKAGE_DIR, "msg")
SRV_DIR = os.path.join(PACKAGE_DIR, "srv")

FORBIDDEN_RAW_TYPES = [
    r"\bfloat64\[\]",
    r"\bfloat32\[\]",
    r"\bint32\[\]",
    r"\buint32\[\]",
    r"\bpx4_msgs/",
    r"\b" + "Trajectory" + "Setpoint",
    r"\b" + "Offboard" + "Control" + "Mode",
]


def get_all_msg_files():
    if not os.path.isdir(MSG_DIR):
        return []
    return [os.path.join(MSG_DIR, f) for f in os.listdir(MSG_DIR) if f.endswith(".msg")]


def get_all_srv_files():
    if not os.path.isdir(SRV_DIR):
        return []
    return [os.path.join(SRV_DIR, f) for f in os.listdir(SRV_DIR) if f.endswith(".srv")]


def parse_definition_lines(filepath):
    lines = []
    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            # Remove comments
            if "#" in stripped:
                stripped = stripped.split("#")[0].strip()
            if stripped:
                lines.append(stripped)
    return lines


class TestRosInterfaceBoundary:
    """Property 21: Concrete ROS interface boundary."""

    def test_msg_files_exist(self):
        msg_files = get_all_msg_files()
        assert len(msg_files) >= 10, f"Expected at least 10 .msg files, found {len(msg_files)}"

    def test_srv_files_exist(self):
        srv_files = get_all_srv_files()
        assert len(srv_files) >= 4, f"Expected at least 4 .srv files, found {len(srv_files)}"

    @pytest.mark.parametrize("msg_file", get_all_msg_files())
    def test_msg_string_and_array_bounds(self, msg_file):
        lines = parse_definition_lines(msg_file)
        basename = os.path.basename(msg_file)

        for line in lines:
            # Check for constant definitions (e.g. uint8 CONST_NAME=1)
            if "=" in line:
                continue

            parts = line.split()
            if not parts:
                continue
            type_name = parts[0]

            # 1. Unbounded string check
            if type_name == "string":
                pytest.fail(f"Unbounded string found in {basename}: '{line}'. Must be string<=N.")

            # 2. Unbounded array check (e.g. type[])
            if type_name.endswith("[]") and not type_name.endswith("[36]"):
                pytest.fail(f"Unbounded sequence found in {basename}: '{line}'. Must have max size bound [<=N].")

            # 3. Forbidden raw types
            for forbidden_pat in FORBIDDEN_RAW_TYPES:
                if re.search(forbidden_pat, type_name):
                    pytest.fail(f"Forbidden raw type found in {basename}: '{line}' matching {forbidden_pat}")

    @pytest.mark.parametrize("srv_file", get_all_srv_files())
    def test_srv_boundary_and_structure(self, srv_file):
        lines = parse_definition_lines(srv_file)
        basename = os.path.basename(srv_file)

        # Must have separator ---
        assert "---" in lines, f"Service file {basename} missing '---' separator."

        for line in lines:
            if line == "---" or "=" in line:
                continue

            parts = line.split()
            if not parts:
                continue
            type_name = parts[0]

            # 1. Unbounded string check
            if type_name == "string":
                pytest.fail(f"Unbounded string found in {basename}: '{line}'. Must be string<=N.")

            # 2. Unbounded array check
            if type_name.endswith("[]"):
                pytest.fail(f"Unbounded sequence found in {basename}: '{line}'. Must have max size bound [<=N].")

            # 3. Forbidden raw types
            for forbidden_pat in FORBIDDEN_RAW_TYPES:
                if re.search(forbidden_pat, type_name):
                    pytest.fail(f"Forbidden raw type found in {basename}: '{line}' matching {forbidden_pat}")
