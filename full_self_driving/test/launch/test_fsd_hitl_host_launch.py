import os
import pytest
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
import importlib.util


def test_fsd_hitl_host_launch_description():
    launch_path = os.path.join(
        os.path.dirname(__file__), "..", "..", "launch", "fsd_hitl_host.launch.py"
    )
    assert os.path.exists(launch_path), f"Launch file not found at {launch_path}"

    spec = importlib.util.spec_from_file_location("fsd_hitl_host", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)

    declared_args = [a.name for a in ld.entities if isinstance(a, DeclareLaunchArgument)]
    assert "world" in declared_args
    assert "headless" in declared_args
    assert "foxglove_port" in declared_args
