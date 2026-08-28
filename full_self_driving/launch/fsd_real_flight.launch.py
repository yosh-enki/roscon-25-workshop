#!/usr/bin/env python3

# ==============================================================================
# Full Self Driving (FSD) - Authoritative Real Hardware Flight Launch
# Target: Raspberry Pi 4 Companion Computer & Pixhawk Autopilot (Physical Drone)
# Pure Standalone FSD Architecture (Zero Prototype Dependencies)
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
    camera_driver = LaunchConfiguration("camera_driver").perform(context).strip().lower()
    camera_fps_val = float(LaunchConfiguration("camera_fps").perform(context).strip())
    camera_pixel_format = LaunchConfiguration("camera_pixel_format").perform(context).strip()
    camera_info_url_arg = LaunchConfiguration("camera_info_url").perform(context).strip()
    world_name = LaunchConfiguration("world").perform(context).strip()
    payload_adapter = LaunchConfiguration("payload_adapter").perform(context).strip()
    gripper_inst = int(LaunchConfiguration("gripper_instance").perform(context).strip())
    dictionary_name = LaunchConfiguration("dictionary").perform(context).strip()
    marker_size_val = float(LaunchConfiguration("marker_size").perform(context).strip())
    target_marker_id = int(LaunchConfiguration("target_marker_id").perform(context).strip())
    target_namespace = LaunchConfiguration("target_namespace").perform(context).strip()
    hardware_manifest_arg = LaunchConfiguration("hardware_manifest").perform(context).strip()
    config_path_arg = LaunchConfiguration("engineering_config").perform(context).strip()
    global_pos_topic = LaunchConfiguration("global_position_topic").perform(context).strip()
    use_sim_time = False  # Strictly False for real hardware flight

    try:
        pkg_share = FindPackageShare("full_self_driving").find("full_self_driving")
    except Exception:
        pkg_share = ""

    # Resolve Camera Calibration URL
    if not camera_info_url_arg:
        calib_candidates = []
        if pkg_share:
            calib_candidates.append(os.path.join(pkg_share, "config", "camera_calibrations", "c270_720p.yaml"))
        calib_candidates.extend([
            os.path.expanduser("~/.ros/camera_info/c270.yaml"),
            "/root/.ros/camera_info/c270.yaml",
        ])
        for c in calib_candidates:
            if os.path.exists(c):
                camera_info_url_arg = f"file://{c}"
                break

    # 1. Resolve Authoritative Config
    if not config_path_arg and pkg_share:
        for c in [
            os.path.join(pkg_share, "config", "fsd_parameters_real.yaml"),
            os.path.join(pkg_share, "config", "fsd_parameters.yaml"),
        ]:
            if os.path.exists(c):
                config_path_arg = c
                break

    # If authoritative config exists, extract perception defaults if not overridden
    if config_path_arg and os.path.exists(config_path_arg):
        try:
            with open(config_path_arg, "r", encoding="utf-8") as f:
                cfg_yaml = yaml.safe_load(f) or {}
            perc_cfg = cfg_yaml.get("perception", {})
            if "dictionary" in perc_cfg and LaunchConfiguration("dictionary").perform(context) == "DICT_4X4_50":
                dictionary_name = perc_cfg["dictionary"]
            if "marker_size_m" in perc_cfg and LaunchConfiguration("marker_size").perform(context) == "0.40":
                marker_size_val = float(perc_cfg["marker_size_m"])
        except Exception as exc:
            print(f"[WARN] Error extracting parameters from {config_path_arg}: {exc}", file=sys.stderr)

    # 2. Resolve Hardware Manifest
    if not hardware_manifest_arg and pkg_share:
        default_manifest = os.path.join(pkg_share, "config", "manifests", "hitl_rpi4_pixhawk.yaml")
        if os.path.exists(default_manifest):
            hardware_manifest_arg = default_manifest

    entities = [
        LogInfo(msg="==============================================================="),
        LogInfo(msg=" 🚁 Starting FSD Autonomous Real Hardware Flight Stack"),
        LogInfo(msg=f"    • Serial Port:     {serial_port} @ {baud_rate} baud"),
        LogInfo(msg=f"    • MicroXRCEAgent:  {'INTERNAL (Launch managed)' if start_agent else 'EXTERNAL (Daemon managed via ./run_raspi.sh)'}"),
        LogInfo(msg=f"    • Config:          {config_path_arg}"),
        LogInfo(msg=f"    • Manifest:        {hardware_manifest_arg}"),
        LogInfo(msg=f"    • World / Map:     {world_name}"),
        LogInfo(msg=f"    • Target Marker:   ID {target_marker_id} ({dictionary_name}, {marker_size_val}m)"),
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
    else:
        entities.append(LogInfo(msg="ℹ️ MicroXRCEAgent spawn skipped (start_agent:=false). Assuming active daemon on " + serial_port))

    # --------------------------------------------------------------------------
    # B. Transforms (TF Tree: map -> odom -> base_link -> camera_frame)
    # --------------------------------------------------------------------------
    map_to_odom_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf_publisher",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )
    entities.append(map_to_odom_tf_node)

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
            "--x", "0.10",
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
    # C. Camera Driver (Real USB Logitech C270 / V4L2 Camera)
    # --------------------------------------------------------------------------
    if start_camera:
        if camera_driver == "usb_cam":
            usb_cam_params = {
                "video_device": camera_device,
                "image_width": 1280,
                "image_height": 720,
                "framerate": camera_fps_val,
                "pixel_format": camera_pixel_format,
                "camera_name": "c270",
                "frame_id": "camera_frame",
                "use_sim_time": use_sim_time,
            }
            if camera_info_url_arg:
                usb_cam_params["camera_info_url"] = camera_info_url_arg

            camera_node = Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                output="screen",
                parameters=[usb_cam_params],
                remappings=[
                    ("image_raw", "/camera"),
                    ("camera_info", "/camera_info"),
                ],
            )
            entities.append(camera_node)
        else:
            v4l2_camera_node = Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                output="screen",
                parameters=[{
                    "video_device": camera_device,
                    "image_size": [1280, 720],
                    "camera_frame_id": "camera_frame",
                    "pixel_format": camera_pixel_format if camera_pixel_format != "mjpeg2rgb" else "YUYV",
                    "output_encoding": "rgb8",
                    "use_sim_time": use_sim_time,
                }],
                remappings=[
                    ("image_raw", "/camera"),
                    ("camera_info", "/camera_info"),
                ],
            )
            entities.append(v4l2_camera_node)

    # --------------------------------------------------------------------------
    # D. Foxglove Studio WebSocket Bridge (GCS Telemetry & Mission Control)
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
    # E. FSD Autonomy Stack (Health, Registry, Perception, Gateway, Runtime)
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
            "scenario_id": "default_scenario",
            "global_position_topic": global_pos_topic,
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
            "scenario_id": "default_scenario",
            "target_namespace": target_namespace,
            "dictionary": dictionary_name,
            "marker_size": marker_size_val,
            "min_quality": 0.0,
            "selected_marker_id": target_marker_id,
            "selected_dictionary": dictionary_name,
            "selected_namespace": target_namespace,
            "lock_min_consecutive_observations": 1,
            "lock_spatial_consistency_radius_m": 25.0,
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
            default_value="true",
            description="Whether to start MicroXRCEAgent inside this launch (false if using ./run_raspi.sh start daemon)",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Whether to start camera driver node for physical camera",
        ),
        DeclareLaunchArgument(
            "camera_driver",
            default_value="usb_cam",
            description="Camera driver type: 'usb_cam' (recommended, 30fps mjpeg) or 'v4l2_camera'",
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="Linux video device path for camera (Default: /dev/video0)",
        ),
        DeclareLaunchArgument(
            "camera_fps",
            default_value="30.0",
            description="Camera capture framerate (Default: 30.0)",
        ),
        DeclareLaunchArgument(
            "camera_pixel_format",
            default_value="mjpeg2rgb",
            description="Camera pixel format ('mjpeg2rgb' for usb_cam, 'YUYV' for v4l2_camera)",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="",
            description="URL or path to camera calibration file (e.g. file:///root/.ros/camera_info/c270.yaml)",
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
            default_value="8",
            description="Target ArUco Marker ID to search and precision-land on",
        ),
        DeclareLaunchArgument(
            "target_namespace",
            default_value="aavc2026",
            description="Target mission namespace",
        ),
        DeclareLaunchArgument(
            "global_position_topic",
            default_value="/fmu/out/vehicle_global_position",
            description="PX4 fused global position topic (/fmu/out/vehicle_global_position or /fmu/out/vehicle_gps_position)",
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
