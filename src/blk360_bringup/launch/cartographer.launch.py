"""Cartographer 2D SLAM bringup.

Runs cartographer_node (scan matching, map->odom TF) and
cartographer_occupancy_grid_node (publishes /map as nav_msgs/OccupancyGrid,
which Nav2 and frontier_exploration_ros2 consume).
"""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("blk360_bringup")
    config_dir = str(Path(pkg_share) / "config" / "cartographer")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_basename = LaunchConfiguration("config_basename")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("config_basename", default_value="cartographer_2d.lua"),
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),

        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", config_basename,
            ],
            remappings=[("scan", scan_topic)],
        ),

        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-resolution", resolution,
                "-publish_period_sec", publish_period_sec,
            ],
        ),
    ])
