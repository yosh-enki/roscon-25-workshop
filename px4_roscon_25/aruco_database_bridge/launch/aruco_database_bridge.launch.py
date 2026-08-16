import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("aruco_database_bridge").find("aruco_database_bridge")
    parameter_file = os.path.join(package_share, "config", "params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the simulation clock for ROS timestamps",
        ),
        DeclareLaunchArgument(
            "mqtt_host",
            default_value="",
            description="MQTT broker hostname without a URL scheme",
        ),
        DeclareLaunchArgument(
            "mqtt_port",
            default_value="8883",
            description="MQTT TLS port",
        ),
        DeclareLaunchArgument(
            "mqtt_username",
            default_value="",
            description="MQTT username; leave empty to use the environment",
        ),
        DeclareLaunchArgument(
            "mqtt_password",
            default_value="",
            description="MQTT password; leave empty to use the environment",
        ),
        DeclareLaunchArgument(
            "mqtt_client_id",
            default_value="aruco_database_bridge",
            description="Unique MQTT client ID",
        ),
        DeclareLaunchArgument(
            "mqtt_topic_prefix",
            default_value="aruco_database",
            description="Concrete MQTT topic prefix",
        ),
        DeclareLaunchArgument(
            "mqtt_tls_ca_file",
            default_value="",
            description="Optional custom CA file; empty uses system CAs",
        ),
        DeclareLaunchArgument(
            "status_publish_period_ms",
            default_value="1000",
            description="Periodic retained status publish period",
        ),
        DeclareLaunchArgument(
            "core_status_timeout_ms",
            default_value="5000",
            description="Maximum age of a core status heartbeat",
        ),
        Node(
            package="aruco_database_bridge",
            executable="aruco_database_bridge",
            name="aruco_database_bridge",
            output="screen",
            parameters=[
                parameter_file,
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "mqtt_host": LaunchConfiguration("mqtt_host"),
                    "mqtt_port": ParameterValue(
                        LaunchConfiguration("mqtt_port"), value_type=int
                    ),
                    "mqtt_username": LaunchConfiguration("mqtt_username"),
                    "mqtt_password": LaunchConfiguration("mqtt_password"),
                    "mqtt_client_id": LaunchConfiguration("mqtt_client_id"),
                    "mqtt_topic_prefix": LaunchConfiguration("mqtt_topic_prefix"),
                    "mqtt_tls_ca_file": LaunchConfiguration("mqtt_tls_ca_file"),
                    "status_publish_period_ms": ParameterValue(
                        LaunchConfiguration("status_publish_period_ms"), value_type=int
                    ),
                    "core_status_timeout_ms": ParameterValue(
                        LaunchConfiguration("core_status_timeout_ms"), value_type=int
                    ),
                },
            ],
        ),
    ])
