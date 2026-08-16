import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("aruco_database").find("aruco_database")
    parameter_file = os.path.join(package_share, "config", "params.yaml")
    default_database_file = os.path.join(package_share, "database", "markers.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_file",
            default_value=default_database_file,
            description="Persistent ArUco database file in the package database directory",
        ),
        DeclareLaunchArgument(
            "detection_topic",
            default_value="/aruco/detections",
            description="All-marker detection topic from aruco_tracker",
        ),
        DeclareLaunchArgument(
            "global_position_topic",
            default_value="/fmu/out/vehicle_global_position",
            description="PX4 fused global-position topic used for auto-origin",
        ),
        DeclareLaunchArgument(
            "vehicle_frame",
            default_value="base_link_frd",
            description="Vehicle frame used to capture the local launch position",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "auto_origin",
            default_value="true",
            description="Latch the first valid vehicle GPS position as the launch origin",
        ),
        DeclareLaunchArgument(
            "origin_configured",
            default_value="false",
            description="Use the manually configured WGS84 origin instead of auto-origin",
        ),
        DeclareLaunchArgument(
            "origin_latitude_deg",
            default_value="0.0",
            description="Latitude of the odom_ned origin in degrees",
        ),
        DeclareLaunchArgument(
            "origin_longitude_deg",
            default_value="0.0",
            description="Longitude of the odom_ned origin in degrees",
        ),
        DeclareLaunchArgument(
            "world_frame",
            default_value="odom_ned",
            description="Frame whose x/y axes are North/East in metres",
        ),
        DeclareLaunchArgument(
            "save_period_s",
            default_value="2.0",
            description="Fallback database persistence period in seconds",
        ),
        DeclareLaunchArgument(
            "save_on_update",
            default_value="true",
            description="Persist accepted detections without waiting for the timer",
        ),
        DeclareLaunchArgument(
            "save_min_interval_ms",
            default_value="200",
            description="Minimum interval between update-triggered file writes",
        ),
        DeclareLaunchArgument(
            "transform_timeout_s",
            default_value="0.05",
            description="TF lookup timeout in seconds",
        ),
        Node(
            package="aruco_database",
            executable="aruco_database_node",
            name="aruco_database",
            output="screen",
            parameters=[
                parameter_file,
                {
                    "database_file": LaunchConfiguration("database_file"),
                    "detection_topic": LaunchConfiguration("detection_topic"),
                    "global_position_topic": LaunchConfiguration("global_position_topic"),
                    "vehicle_frame": LaunchConfiguration("vehicle_frame"),
                    "auto_origin": ParameterValue(
                        LaunchConfiguration("auto_origin"), value_type=bool
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                    "origin_configured": ParameterValue(
                        LaunchConfiguration("origin_configured"), value_type=bool
                    ),
                    "origin_latitude_deg": ParameterValue(
                        LaunchConfiguration("origin_latitude_deg"), value_type=float
                    ),
                    "origin_longitude_deg": ParameterValue(
                        LaunchConfiguration("origin_longitude_deg"), value_type=float
                    ),
                    "world_frame": LaunchConfiguration("world_frame"),
                    "save_period_s": ParameterValue(
                        LaunchConfiguration("save_period_s"), value_type=float
                    ),
                    "save_on_update": ParameterValue(
                        LaunchConfiguration("save_on_update"), value_type=bool
                    ),
                    "save_min_interval_ms": ParameterValue(
                        LaunchConfiguration("save_min_interval_ms"), value_type=int
                    ),
                    "transform_timeout_s": ParameterValue(
                        LaunchConfiguration("transform_timeout_s"), value_type=float
                    ),
                },
            ],
        ),
    ])
