#!/usr/bin/env python3

import os
import sys
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    simulation_arg = LaunchConfiguration("simulation").perform(context).lower()
    world_name = LaunchConfiguration("world").perform(context)
    headless_arg = LaunchConfiguration("headless").perform(context).lower()
    config_path = LaunchConfiguration("engineering_config").perform(context)
    test_fault_child = LaunchConfiguration("test_fault_child").perform(context)
    replay_fixture = LaunchConfiguration("replay_fixture").perform(context).lower()
    dictionary_name = LaunchConfiguration("dictionary").perform(context)
    marker_size_val = float(LaunchConfiguration("marker_size").perform(context))

    simulation = simulation_arg in ["true", "1", "yes"]
    headless = headless_arg in ["true", "1", "yes"]

    # Explicit Hardware Profile Deferral Gate
    if not simulation:
        print(
            "\n[ERROR] HARDWARE_PROFILE_NOT_CONFIGURED: Hardware bringup for Raspberry Pi 4 is "
            "explicitly deferred pending an approved hardware manifest and validation evidence.\n",
            file=sys.stderr,
        )
        raise RuntimeError("HARDWARE_PROFILE_NOT_CONFIGURED")

    pkg_share = FindPackageShare("full_self_driving").find("full_self_driving")

    # Load simulation manifests
    profile_manifest_file = os.path.join(pkg_share, "simulation", "manifests", "profile_simulation.yaml")
    world_manifest_file = os.path.join(pkg_share, "simulation", "manifests", f"{world_name}.yaml")

    if not os.path.exists(profile_manifest_file):
        raise RuntimeError(f"Simulation profile manifest not found: {profile_manifest_file}")
    if not os.path.exists(world_manifest_file):
        raise RuntimeError(f"World manifest not found: {world_manifest_file}")

    with open(profile_manifest_file, "r", encoding="utf-8") as f:
        profile_manifest = yaml.safe_load(f)
    with open(world_manifest_file, "r", encoding="utf-8") as f:
        world_manifest = yaml.safe_load(f)

    if world_name not in profile_manifest.get("allowed_worlds", []):
        raise RuntimeError(f"World '{world_name}' is not in allowed_worlds list")

    world_sdf_rel = world_manifest.get("world_sdf", f"simulation/worlds/{world_name}.sdf")
    world_sdf = os.path.join(pkg_share, world_sdf_rel)
    if not os.path.exists(world_sdf):
        raise RuntimeError(f"World SDF not found: {world_sdf}")

    model_name = world_manifest.get("vehicle", {}).get("model_name", "x500_mono_cam_down_0")
    camera_frame = world_manifest.get("vehicle", {}).get("camera_frame", "camera_frame")
    model_camera_frame = world_manifest.get("vehicle", {}).get("model_camera_frame", f"{model_name}/camera_link/imager")
    airframe_id = str(world_manifest.get("vehicle", {}).get("airframe", 4001))

    # Bridge files
    clock_bridge_config_file = os.path.join(pkg_share, "simulation", "bridges", "clock.yaml")
    camera_bridge_template_file = os.path.join(pkg_share, "simulation", "bridges", "camera.yaml")

    with open(camera_bridge_template_file, "r", encoding="utf-8") as f:
        camera_bridge_config = yaml.safe_load(f)

    for item in camera_bridge_config:
        item["gz_topic_name"] = item["gz_topic_name"].format(
            world_name=world_name,
            model_name=model_name,
        )

    tmp_camera_bridge = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False)
    with open(tmp_camera_bridge.name, "w", encoding="utf-8") as f:
        yaml.dump(camera_bridge_config, f)

    camera_gz_topic = f"/world/{world_name}/model/{model_name}/link/camera_link/sensor/imager/image"

    # URDF
    urdf_file = os.path.join(pkg_share, "simulation", "urdf", "x500.urdf")
    with open(urdf_file, "r", encoding="utf-8") as infp:
        x500_desc = infp.read()

    # Environment for Gazebo
    gz_models_dir = profile_manifest.get("gazebo", {}).get("model_store", "/home/ubuntu/PX4-gazebo-models/models")
    gz_server_config = profile_manifest.get("gazebo", {}).get("server_config_path", "/home/ubuntu/PX4-gazebo-models/server.config")
    worlds_dir = os.path.join(pkg_share, "simulation", "worlds")

    gz_env = {
        "GZ_SIM_RESOURCE_PATH": f"{gz_models_dir}:{worlds_dir}",
        "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config,
    }

    # Gazebo command
    gz_cmd = ["gz", "sim", "-r", world_sdf]
    if headless:
        gz_cmd.append("-s")

    gz_process = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env=gz_env,
    )

    # PX4 SITL command
    px4_bin = profile_manifest.get("px4_sitl", {}).get("executable", "/home/ubuntu/px4_sitl/bin/px4")
    px4_romfs = profile_manifest.get("px4_sitl", {}).get("working_dir", "/home/ubuntu/px4_sitl/romfs")
    px4_model = profile_manifest.get("px4_sitl", {}).get("model_default", "x500_mono_cam_down")
    px4_env = {
        "PX4_GZ_STANDALONE": "1",
        "PX4_SYS_AUTOSTART": airframe_id,
        "PX4_PARAM_UXRCE_DDS_SYNCT": "0",
        "PX4_SIM_MODEL": px4_model,
    }
    px4_process = ExecuteProcess(
        cmd=[px4_bin, "-w", px4_romfs],
        output="screen",
        additional_env=px4_env,
    )

    # MicroXRCE DDS Agent
    dds_agent_process = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888", "-v", "3"],
        output="screen",
    )

    # Bridges & Nodes
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        parameters=[{"config_file": clock_bridge_config_file}],
    )

    camera_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_info_bridge",
        parameters=[{"config_file": tmp_camera_bridge.name}],
    )

    image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="camera_image_bridge",
        output="screen",
        arguments=[camera_gz_topic],
        remappings=[
            (camera_gz_topic, "/camera"),
            (f"{camera_gz_topic}/compressed", "/camera/compressed"),
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": x500_desc,
            "use_sim_time": True,
        }],
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{"use_sim_time": True}],
    )

    px4_tf_node = Node(
        package="px4_tf",
        executable="px4_tf_publisher",
        name="px4_tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    tf_container = ComposableNodeContainer(
        name="static_tf_container",
        package="rclcpp_components",
        executable="component_container",
        namespace="",
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="map_to_odom_broadcaster",
                parameters=[{
                    "use_sim_time": True,
                    "translation.x": 0.0,
                    "translation.y": 0.0,
                    "translation.z": 0.24,
                    "rotation.x": 0.0,
                    "rotation.y": 0.0,
                    "rotation.z": 0.0,
                    "frame_id": "map",
                    "child_frame_id": "odom",
                }],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="base_link_to_camera_broadcaster",
                parameters=[{
                    "use_sim_time": True,
                    "translation.x": 0.0,
                    "translation.y": 0.0,
                    "translation.z": 0.0,
                    "rotation.x": -0.7071068,
                    "rotation.y": 0.7071068,
                    "rotation.z": 0.0,
                    "rotation.w": 0.0,
                    "frame_id": "base_link",
                    "child_frame_id": model_camera_frame,
                }],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="camera_to_camera_frame_broadcaster",
                parameters=[{
                    "use_sim_time": True,
                    "translation.x": 0.0,
                    "translation.y": 0.0,
                    "translation.z": 0.0,
                    "rotation.x": 0.0,
                    "rotation.y": 0.0,
                    "rotation.z": 0.0,
                    "rotation.w": 1.0,
                    "frame_id": model_camera_frame,
                    "child_frame_id": camera_frame,
                }],
            ),
        ],
    )

    launch_probe_node = Node(
        package="full_self_driving",
        executable="fsd_launch_probe",
        name="fsd_launch_probe",
        output="screen",
        parameters=[{
            "simulation": True,
            "world": world_name,
            "engineering_config": config_path,
            "use_sim_time": True,
        }],
    )

    test_selection = LaunchConfiguration("test_selection").perform(context).strip()
    property_fixture = LaunchConfiguration("property_fixture").perform(context).strip().lower()

    selected_marker_id = -1
    selected_dict = dictionary_name
    selected_ns = "aavc2026"
    if test_selection and test_selection.lower() != "none":
        parts = test_selection.split(":")
        try:
            selected_marker_id = int(parts[0])
            if len(parts) > 1 and parts[1]:
                selected_dict = parts[1]
            if len(parts) > 2 and parts[2]:
                selected_ns = parts[2]
        except ValueError:
            pass

    fsd_perception_node = Node(
        package="full_self_driving",
        executable="fsd_perception",
        name="fsd_perception",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "camera_frame": camera_frame,
            "map_id": world_name,
            "scenario_id": "default_scenario",
            "target_namespace": "aavc2026",
            "dictionary": dictionary_name,
            "marker_size": marker_size_val,
            "selected_marker_id": selected_marker_id,
            "selected_dictionary": selected_dict,
            "selected_namespace": selected_ns,
            "autostart": True,
        }],
    )

    fsd_pad_registry_node = Node(
        package="full_self_driving",
        executable="fsd_pad_registry",
        name="fsd_pad_registry",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "map_id": world_name,
            "scenario_id": "default_scenario",
            "autostart": True,
        }],
    )

    fsd_evidence_node = Node(
        package="full_self_driving",
        executable="fsd_evidence",
        name="fsd_evidence",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "evidence_directory": "/tmp/fsd_evidence",
            "autostart": True,
        }],
    )

    fsd_gateway_node = Node(
        package="full_self_driving",
        executable="fsd_gateway",
        name="fsd_gateway",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
        }],
    )

    entities = [
        gz_process,
        px4_process,
        dds_agent_process,
        clock_bridge_node,
        camera_bridge_node,
        image_bridge_node,
        robot_state_publisher_node,
        px4_tf_node,
        foxglove_bridge_node,
        tf_container,
        launch_probe_node,
        fsd_pad_registry_node,
        fsd_perception_node,
        fsd_evidence_node,
        fsd_gateway_node,
    ]

    if test_selection and test_selection.lower() != "none" and selected_marker_id >= 0:
        selection_provider_node = Node(
            package="full_self_driving",
            executable="fsd_target_selection_provider",
            name="fsd_target_selection_provider",
            output="screen",
            parameters=[{
                "marker_id": selected_marker_id,
                "dictionary": selected_dict,
                "target_namespace": selected_ns,
                "rate_hz": 1.0,
                "periodic": True,
                "use_sim_time": True,
            }],
        )
        entities.append(selection_provider_node)

    if replay_fixture == "aruco":
        replay_publisher_node = Node(
            package="full_self_driving",
            executable="fsd_replay_fixture_publisher",
            name="fsd_replay_fixture_publisher",
            output="screen",
            parameters=[{
                "fixture_name": "aruco",
                "rate_hz": 10.0,
                "use_sim_time": True,
            }],
        )
        entities.append(replay_publisher_node)

    # Supervise child exits: if gz or px4 or dds agent exits, initiate shutdown
    entities.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_process,
                on_exit=[EmitEvent(event=Shutdown(reason="Gazebo simulation process exited"))],
            )
        )
    )
    entities.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=px4_process,
                on_exit=[EmitEvent(event=Shutdown(reason="PX4 SITL process exited"))],
            )
        )
    )

    if test_fault_child == "px4":
        fault_process = ExecuteProcess(
            cmd=["bash", "-c", "sleep 3 && kill -9 $(pgrep -f px4_sitl/bin/px4 || true)"],
            output="screen",
        )
        entities.append(fault_process)

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "simulation",
            default_value="true",
            description="Launch simulation profile (true) or hardware profile (false)",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="kmitl_airfield",
            description="Allowlisted Gazebo world name",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo in headless mode without GUI",
        ),
        DeclareLaunchArgument(
            "engineering_config",
            default_value="",
            description="Path to authoritative engineering configuration file",
        ),
        DeclareLaunchArgument(
            "replay_fixture",
            default_value="none",
            description="Replay fixture test mode ('none', 'aruco')",
        ),
        DeclareLaunchArgument(
            "test_selection",
            default_value="none",
            description="Test-only target selection fixture (e.g. '0', '7', '0:DICT_4X4_50:aavc2026')",
        ),
        DeclareLaunchArgument(
            "property_fixture",
            default_value="none",
            description="Property test fixture mode ('none', 'registry_isolation', 'all_id_live_lock')",
        ),
        DeclareLaunchArgument(
            "dictionary",
            default_value="DICT_4X4_50",
            description="ArUco marker dictionary name (e.g. DICT_4X4_50, DICT_4X4_250)",
        ),
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.4",
            description="Marker side length in meters",
        ),
        DeclareLaunchArgument(
            "test_fault_child",
            default_value="none",
            description="Fault injection testing argument ('none' or child name)",
        ),
        OpaqueFunction(function=launch_setup),
    ])
