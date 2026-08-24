#!/usr/bin/env python3

import os
import sys
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    serial_port = LaunchConfiguration("serial_port").perform(context).strip()
    baud_rate = LaunchConfiguration("baud_rate").perform(context).strip()
    start_agent_arg = LaunchConfiguration("start_agent").perform(context).lower()
    world_name = LaunchConfiguration("world").perform(context).strip()
    payload_adapter = LaunchConfiguration("payload_adapter").perform(context).strip()
    gripper_inst = int(LaunchConfiguration("gripper_instance").perform(context).strip())
    dictionary_name = LaunchConfiguration("dictionary").perform(context).strip()
    marker_size_val = float(LaunchConfiguration("marker_size").perform(context).strip())
    hardware_manifest_arg = LaunchConfiguration("hardware_manifest").perform(context).strip()
    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context).lower()

    start_agent = start_agent_arg in ["true", "1", "yes"]
    use_sim_time = use_sim_time_arg in ["true", "1", "yes"]

    pkg_share = FindPackageShare("full_self_driving").find("full_self_driving")

    # Resolve Hardware Manifest
    if not hardware_manifest_arg:
        default_manifest = os.path.join(pkg_share, "config", "manifests", "hitl_rpi4_pixhawk.yaml")
        if os.path.exists(default_manifest):
            hardware_manifest_arg = default_manifest

    # Resolve Engineering Config
    config_path = os.path.join(pkg_share, "config", "fsd_parameters.yaml")

    entities = []

    # 1. MicroXRCEAgent Serial Transport (Pixhawk TELEM2 link)
    if start_agent:
        dds_agent_process = ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", serial_port, "-b", baud_rate, "-v", "3"],
            output="screen",
        )
        entities.append(dds_agent_process)

    # 2. PX4 TF Publisher (odom -> base_link from vehicle_odometry)
    px4_tf_node = Node(
        package="px4_tf",
        executable="px4_tf_publisher",
        name="px4_tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    entities.append(px4_tf_node)

    # 3. Launch Probe (Health & Readiness)
    launch_probe_node = Node(
        package="full_self_driving",
        executable="fsd_launch_probe",
        name="fsd_launch_probe",
        output="screen",
        parameters=[{
            "simulation": False,
            "world": world_name,
            "engineering_config": config_path if os.path.exists(config_path) else "",
            "use_sim_time": use_sim_time,
        }],
    )
    entities.append(launch_probe_node)

    # 4. Delivery Pad Registry
    fsd_pad_registry_node = Node(
        package="full_self_driving",
        executable="fsd_pad_registry",
        name="fsd_pad_registry",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_id": world_name,
            "scenario_id": "default_scenario",
            "autostart": True,
        }],
    )
    entities.append(fsd_pad_registry_node)

    # 5. ArUco Perception Lifecycle Node
    fsd_perception_node = Node(
        package="full_self_driving",
        executable="fsd_perception",
        name="fsd_perception",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "camera_frame": "camera_frame",
            "map_id": world_name,
            "scenario_id": "default_scenario",
            "target_namespace": "aavc2026",
            "dictionary": dictionary_name,
            "marker_size": marker_size_val,
            "selected_marker_id": -1,
            "selected_dictionary": dictionary_name,
            "selected_namespace": "aavc2026",
            "lock_min_consecutive_observations": 1,
            "lock_spatial_consistency_radius_m": 25.0,
            "autostart": True,
        }],
    )
    entities.append(fsd_perception_node)

    # 6. Post-Drop Visual Evidence & Journaling
    fsd_evidence_node = Node(
        package="full_self_driving",
        executable="fsd_evidence",
        name="fsd_evidence",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "evidence_directory": "/tmp/fsd_evidence",
            "autostart": True,
        }],
    )
    entities.append(fsd_evidence_node)

    # 7. Gateway Boundary Gatekeeper
    fsd_gateway_node = Node(
        package="full_self_driving",
        executable="fsd_gateway",
        name="fsd_gateway",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
        }],
    )
    entities.append(fsd_gateway_node)

    # 8. Flight Runtime & Autonomous Sortie Executor
    fsd_flight_runtime_node = Node(
        package="full_self_driving",
        executable="fsd_flight_runtime",
        name="fsd_flight_runtime",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "engineering_config": config_path if os.path.exists(config_path) else "",
            "simulation": False,
            "world": world_name,
            "target_marker_id": 0,
            "target_dictionary": dictionary_name,
            "target_namespace": "aavc2026",
            "payload_adapter": payload_adapter,
            "gripper_instance": gripper_inst,
        }],
    )
    entities.append(fsd_flight_runtime_node)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyAMA0",
            description="UART Serial device path connecting Pixhawk TELEM2",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="921600",
            description="Serial baud rate (921600 bps)",
        ),
        DeclareLaunchArgument(
            "start_agent",
            default_value="true",
            description="Automatically spawn MicroXRCEAgent serial daemon",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="kmitl_airfield",
            description="Operational world name",
        ),
        DeclareLaunchArgument(
            "payload_adapter",
            default_value="px4_uorb_gripper_actuator",
            description="Payload adapter type (px4_uorb_gripper_actuator for AUX 1)",
        ),
        DeclareLaunchArgument(
            "gripper_instance",
            default_value="1",
            description="Gripper instance index (1 for AUX 1)",
        ),
        DeclareLaunchArgument(
            "dictionary",
            default_value="DICT_4X4_50",
            description="ArUco marker dictionary name",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.50",
            description="ArUco marker physical size in meters",
        ),
        DeclareLaunchArgument(
            "hardware_manifest",
            default_value="",
            description="Path to approved hardware manifest",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock (/clock from Gazebo over LAN)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
