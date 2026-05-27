from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("device_address", default_value="192.168.10.90:8081"),
        DeclareLaunchArgument("trigger_command", default_value="scan"),
        DeclareLaunchArgument("output_dir", default_value="."),
        DeclareLaunchArgument("point_cloud_density", default_value="medium"),
        DeclareLaunchArgument("panorama_mode", default_value="ldr"),
    ]

    node = Node(
        package="blk360_scanner",
        executable="scan_node",
        name="blk360_scanner",
        output="screen",
        parameters=[{
            "device_address": LaunchConfiguration("device_address"),
            "trigger_command": LaunchConfiguration("trigger_command"),
            "output_dir": LaunchConfiguration("output_dir"),
            "point_cloud_density": LaunchConfiguration("point_cloud_density"),
            "panorama_mode": LaunchConfiguration("panorama_mode"),
        }],
    )

    return LaunchDescription(args + [node])
