import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
import importlib.util


def test_fsd_companion_rpi_launch_description():
    launch_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "launch", "fsd_companion_rpi.launch.py"
    )
    assert os.path.exists(launch_path), f"Launch file not found at {launch_path}"

    spec = importlib.util.spec_from_file_location("fsd_companion_rpi", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    declared_args = [a.name for a in ld.entities if isinstance(a, DeclareLaunchArgument)]
    assert "serial_port" in declared_args
    assert "baud_rate" in declared_args
    assert "payload_adapter" in declared_args
    assert "start_agent" in declared_args
