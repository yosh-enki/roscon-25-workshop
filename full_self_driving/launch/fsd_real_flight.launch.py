#!/usr/bin/env python3

# ==============================================================================
# Full Self Driving (FSD) - Authoritative Real Hardware Flight Launch
# Target: Raspberry Pi 4 Companion Computer & Pixhawk Autopilot (Physical Drone)
# With Persistent ArUco Marker Database (markers.yaml)
# ==============================================================================

import os
import sys
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    # Launch Configurations
    serial_port = LaunchConfiguration("serial_port").perform(context).strip()
    baud_rate = LaunchConfiguration("baud_rate").perform(context).strip()
    start_agent = LaunchConfiguration("start_agent").perform(context).lower() in ["true", "1", "yes"]
    start_camera = LaunchConfiguration("start_camera").perform(context).lower() in ["true", "1", "yes"]
    start_foxglove = LaunchConfiguration("start_foxglove").perform(context).lower() in ["true", "1", "yes"]
    foxglove_port_val = int(LaunchConfiguration("foxglove_port").perform(context).strip())
    camera_device = LaunchConfiguration("camera_device").perform(context).strip()
    world_name = LaunchConfiguration("world").perform(context).strip()
    payload_adapter = LaunchConfiguration("payload_adapter").perform(context).strip()
    gripper_inst = int(LaunchConfiguration("gripper_instance").perform(context).strip())
    dictionary_name = LaunchConfiguration("dictionary").perform(context).strip()
    marker_size_val = float(LaunchConfiguration("marker_size").perform(context).strip())
    target_marker_id = int(LaunchConfiguration("target_marker_id").perform(context).strip())
    target_namespace = LaunchConfiguration("target_namespace").perform(context).strip()
    hardware_manifest_arg = LaunchConfiguration("hardware_manifest").perform(context).strip()
    config_path_arg = LaunchConfiguration("engineering_config").perform(context).strip()
    database_file_arg = LaunchConfiguration("database_file").perform(context).strip()
    use_sim_time = False  # Strictly False for real hardware flight

    pkg_share = FindPackageShare("full_self_driving").find("full_self_driving")

    # 1. Resolve Authoritative Config
    if not config_path_arg:
        candidates = [
            os.path.join(pkg_share, "config", "fsd_parameters_real.yaml"),
            os.path.join(pkg_share, "config", "fsd_parameters.yaml"),
            "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/config/fsd_parameters_real.yaml",
            "/home/yosh/roscon-25-workshop/full_self_driving/config/fsd_parameters_real.yaml",
        ]
        for c in candidates:
            if os.path.exists(c):
                config_path_arg = c
                break

    # 2. Resolve Hardware Manifest
    if not hardware_manifest_arg:
        manifest_candidates = [
            os.path.join(pkg_share, "config", "manifests", "hitl_rpi4_pixhawk.yaml"),
            "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml",
            "/home/yosh/roscon-25-workshop/full_self_driving/config/manifests/hitl_rpi4_pixhawk.yaml",
        ]
        for m in manifest_candidates:
            if os.path.exists(m):
                hardware_manifest_arg = m
                break

    # 3. Resolve Persistent Database File (markers.yaml)
    if not database_file_arg:
        try:
            aruco_db_share = FindPackageShare("aruco_database").find("aruco_database")
            database_file_arg = os.path.join(aruco_db_share, "database", "markers.yaml")
        except Exception:
            database_file_arg = ""

        db_candidates = [
            "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/px4_roscon_25/aruco_database/database/markers.yaml",
            "/home/yosh/roscon-25-workshop/px4_roscon_25/aruco_database/database/markers.yaml",
            database_file_arg,
        ]
        for db_c in db_candidates:
            if db_c and os.path.exists(db_c):
                database_file_arg = db_c
                break

    entities = [
        LogInfo(msg="==============================================================="),
        LogInfo(msg=" 🚁 Starting FSD Autonomous Real Hardware Flight Stack"),
        LogInfo(msg=f"    • Serial Port:     {serial_port} @ {baud_rate} baud"),
        LogInfo(msg=f"    • Config:          {config_path_arg}"),
        LogInfo(msg=f"    • Manifest:        {hardware_manifest_arg}"),
        LogInfo(msg=f"    • Persistent DB:   {database_file_arg}"),
        LogInfo(msg=f"    • World / Map:     {world_name}"),
        LogInfo(msg=f"    • Target Marker:   ID {target_marker_id} ({dictionary_name})"),
        LogInfo(msg="==============================================================="),
    ]

    # --------------------------------------------------------------------------
    # A. MicroXRCEAgent (Pixhawk Serial Link)
    # --------------------------------------------------------------------------
    if start_agent:
        dds_agent_process = ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", serial_port, "-b", baud_rate, "-v", "3"],
            output="screen",
        )
        entities.append(dds_agent_process)

    # --------------------------------------------------------------------------
    # B. Transforms (TF Tree: odom -> base_link -> camera_frame)
    # --------------------------------------------------------------------------
    px4_tf_node = Node(
        package="px4_tf",
        executable="px4_tf_publisher",
        name="px4_tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    entities.append(px4_tf_node)

    camera_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf_publisher",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "-0.05",
            "--qx", "-0.7071068",
            "--qy", "0.7071068",
            "--qz", "0.0",
            "--qw", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_frame",
        ],
    )
    entities.append(camera_static_tf_node)

    # --------------------------------------------------------------------------
    # C. Camera Driver (Real V4L2 / USB / Raspberry Pi Camera)
    # --------------------------------------------------------------------------
    if start_camera:
        v4l2_camera_node = Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera",
            output="screen",
            parameters=[{
                "video_device": camera_device,
                "image_size": [1280, 720],
                "camera_frame_id": "camera_frame",
                "pixel_format": "YUYV",
                "output_encoding": "rgb8",
                "use_sim_time": use_sim_time,
            }],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
        )
        entities.append(v4l2_camera_node)

    # --------------------------------------------------------------------------
    # D. Persistent ArUco Marker Database (markers.yaml) & Tracker
    # --------------------------------------------------------------------------
    if database_file_arg and os.path.exists(database_file_arg):
        aruco_database_node = Node(
            package="aruco_database",
            executable="aruco_database_node",
            name="aruco_database",
            output="screen",
            parameters=[{
                "database_file": database_file_arg,
                "detection_topic": "/aruco/detections",
                "global_position_topic": "/fmu/out/vehicle_global_position",
                "vehicle_frame": "base_link",
                "world_frame": "odom",
                "auto_origin": True,
                "use_sim_time": use_sim_time,
                "save_on_update": True,
                "save_period_s": 2.0,
                "transform_timeout_s": 0.1,
            }],
        )
        entities.append(aruco_database_node)

        # ArUco Tracker (px4_roscon_25)
        aruco_tracker_node = Node(
            package="aruco_tracker",
            executable="aruco_tracker_node",
            name="aruco_tracker",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "target_id": target_marker_id,
                "marker_size": marker_size_val,
                "camera_frame": "camera_frame",
            }],
        )
        entities.append(aruco_tracker_node)

    # --------------------------------------------------------------------------
    # E. Foxglove Studio WebSocket Bridge (GCS Telemetry & Mission Control)
    # --------------------------------------------------------------------------
    if start_foxglove:
        foxglove_bridge_node = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": foxglove_port_val,
                "use_sim_time": use_sim_time,
                "send_buffer_limit": 10000000,
            }],
        )
        entities.append(foxglove_bridge_node)

    # --------------------------------------------------------------------------
    # F. FSD Autonomy Stack (Health, Registry, Perception, Gateway, Runtime)
    # --------------------------------------------------------------------------
    launch_probe_node = Node(
        package="full_self_driving",
        executable="fsd_launch_probe",
        name="fsd_launch_probe",
        output="screen",
        parameters=[{
            "simulation": False,
            "world": world_name,
            "engineering_config": config_path_arg if os.path.exists(config_path_arg) else "",
            "use_sim_time": use_sim_time,
        }],
    )
    entities.append(launch_probe_node)

    fsd_pad_registry_node = Node(
        package="full_self_driving",
        executable="fsd_pad_registry",
        name="fsd_pad_registry",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "map_id": world_name,
            "scenario_id": "real_sortie_scenario",
            "autostart": True,
        }],
    )
    entities.append(fsd_pad_registry_node)

    fsd_perception_node = Node(
        package="full_self_driving",
        executable="fsd_perception",
        name="fsd_perception",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "camera_frame": "camera_frame",
            "map_id": world_name,
            "scenario_id": "real_sortie_scenario",
            "target_namespace": target_namespace,
            "dictionary": dictionary_name,
            "marker_size": marker_size_val,
            "selected_marker_id": target_marker_id,
            "selected_dictionary": dictionary_name,
            "selected_namespace": target_namespace,
            "lock_min_consecutive_observations": 2,
            "lock_spatial_consistency_radius_m": 15.0,
            "autostart": True,
        }],
    )
    entities.append(fsd_perception_node)

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

    fsd_flight_runtime_node = Node(
        package="full_self_driving",
        executable="fsd_flight_runtime",
        name="fsd_flight_runtime",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "engineering_config": config_path_arg if os.path.exists(config_path_arg) else "",
            "simulation": False,
            "world": world_name,
            "target_marker_id": target_marker_id,
            "target_dictionary": dictionary_name,
            "target_namespace": target_namespace,
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
            description="UART Serial device connecting Pixhawk TELEM2 (e.g. /dev/ttyAMA0 or /dev/ttyACM0)",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="921600",
            description="UART Baud rate for PX4 MicroXRCE-DDS bridge (921600 bps)",
        ),
        DeclareLaunchArgument(
            "start_agent",
            default_value="false",
            description="Whether to start MicroXRCEAgent inside this launch (false if using ./run_raspi.sh start daemon)",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="Whether to start v4l2_camera driver node for physical camera",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="Linux video device path for camera (Default: /dev/video0)",
        ),
        DeclareLaunchArgument(
            "start_foxglove",
            default_value="true",
            description="Start Foxglove Studio WebSocket bridge for GCS Mission Control",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove Studio WebSocket port (Default: 8765)",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="kmitl_airfield",
            description="Operational airfield / map name",
        ),
        DeclareLaunchArgument(
            "payload_adapter",
            default_value="px4_uorb_gripper_actuator",
            description="Payload adapter type (px4_uorb_gripper_actuator or esp32_serial_bridge)",
        ),
        DeclareLaunchArgument(
            "gripper_instance",
            default_value="1",
            description="Gripper instance index (1 for AUX 1)",
        ),
        DeclareLaunchArgument(
            "dictionary",
            default_value="DICT_4X4_50",
            description="ArUco marker dictionary name (e.g. DICT_4X4_50)",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.40",
            description="Physical ArUco marker size in meters",
        ),
        DeclareLaunchArgument(
            "target_marker_id",
            default_value="0",
            description="Target ArUco Marker ID to search and precision-land on",
        ),
        DeclareLaunchArgument(
            "target_namespace",
            default_value="aavc2026",
            description="Target mission namespace",
        ),
        DeclareLaunchArgument(
            "database_file",
            default_value="",
            description="Path to persistent ArUco markers.yaml database file",
        ),
        DeclareLaunchArgument(
            "hardware_manifest",
            default_value="",
            description="Path to approved hardware manifest (Default: hitl_rpi4_pixhawk.yaml)",
        ),
        DeclareLaunchArgument(
            "engineering_config",
            default_value="",
            description="Path to engineering parameters yaml (Default: fsd_parameters_real.yaml)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
