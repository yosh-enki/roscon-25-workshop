#!/usr/bin/env python3

import os
import sys
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    world_name = LaunchConfiguration("world").perform(context)
    headless_arg = LaunchConfiguration("headless").perform(context).lower()
    foxglove_port_val = int(LaunchConfiguration("foxglove_port").perform(context))
    headless = headless_arg in ["true", "1", "yes"]

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

    world_sdf_rel = world_manifest.get("world_sdf", f"simulation/worlds/{world_name}.sdf")
    world_sdf = os.path.join(pkg_share, world_sdf_rel)
    if not os.path.exists(world_sdf):
        raise RuntimeError(f"World SDF not found: {world_sdf}")

    model_name = world_manifest.get("vehicle", {}).get("model_name", "x500_mono_cam_down_0")
    camera_frame = world_manifest.get("vehicle", {}).get("camera_frame", "camera_frame")
    model_camera_frame = world_manifest.get("vehicle", {}).get("model_camera_frame", f"{model_name}/camera_link/imager")

    # Bridge configurations
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

    # URDF Robot Description
    urdf_file = os.path.join(pkg_share, "simulation", "urdf", "x500.urdf")
    with open(urdf_file, "r", encoding="utf-8") as infp:
        x500_desc = infp.read()

    # Gazebo Environment
    gz_models_dir = profile_manifest.get("gazebo", {}).get("model_store", "/home/ubuntu/PX4-gazebo-models/models")
    gz_server_config = profile_manifest.get("gazebo", {}).get("server_config_path", "/home/ubuntu/PX4-gazebo-models/server.config")
    worlds_dir = os.path.join(pkg_share, "simulation", "worlds")

    gz_env = {
        "GZ_SIM_RESOURCE_PATH": f"{gz_models_dir}:{worlds_dir}",
        "GZ_SIM_SERVER_CONFIG_PATH": gz_server_config,
    }

    # Gazebo Harmonic Process (Physics & Camera rendering)
    gz_cmd = ["gz", "sim", "-r", world_sdf]
    if headless:
        gz_cmd.append("-s")

    gz_process = ExecuteProcess(
        cmd=gz_cmd,
        output="screen",
        additional_env=gz_env,
    )

    # Clock Bridge (Gazebo -> ROS 2 /clock)
    clock_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        parameters=[{"config_file": clock_bridge_config_file}],
    )

    # Camera Info Bridge
    camera_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_info_bridge",
        parameters=[{"config_file": tmp_camera_bridge.name}],
    )

    # Image Bridge (Gazebo Image -> ROS 2 /camera)
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

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": x500_desc,
            "use_sim_time": True,
        }],
    )

    # Foxglove WebSocket Bridge (for Host Visualizer & Mission Control)
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        parameters=[{
            "port": foxglove_port_val,
            "use_sim_time": True,
        }],
    )

    # Static TF Broadcasters (Coordinate Frames)
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

    return [
        gz_process,
        clock_bridge_node,
        camera_bridge_node,
        image_bridge_node,
        robot_state_publisher_node,
        foxglove_bridge_node,
        tf_container,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="kmitl_airfield",
            description="Gazebo simulation world name",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo in headless mode without GUI",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="WebSocket port for Foxglove Studio Bridge",
        ),
        OpaqueFunction(function=launch_setup),
    ])
