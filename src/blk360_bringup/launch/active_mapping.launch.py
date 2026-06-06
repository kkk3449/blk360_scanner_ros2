"""Full BLK360 active-mapping stack.

Composes everything:
  - Cartographer 2D SLAM + Nav2 + frontier exploration   (exploration.launch.py)
  - the BLK360 scanner: real `blk360_scanner` hardware node, or, for simulation,
    a `mock_blk360_scanner` that fakes the scan status handshake
  - the `blk360_stop_scan` sequencer that stops every `scan_interval_m` metres,
    scans, retries on connection loss, and resumes exploration

The robot itself (TurtleBot3 sim or real hardware) must already be running and
publishing /scan, /odom, and the odom->base_footprint TF.

Examples:
  # simulation (mock scanner)
  ros2 launch blk360_bringup active_mapping.launch.py use_sim_time:=true

  # exercise the reconnect path: first scan fails, retry succeeds
  ros2 launch blk360_bringup active_mapping.launch.py fail_first_n_scans:=1

  # real hardware
  ros2 launch blk360_bringup active_mapping.launch.py \
      use_sim_time:=false use_mock_scanner:=false device_address:=192.168.10.90:8081
"""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("blk360_bringup")
    stop_scan_share = get_package_share_directory("blk360_stop_scan")
    launch_dir = Path(bringup_share) / "launch"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_scanner = LaunchConfiguration("use_mock_scanner")
    scan_interval_m = LaunchConfiguration("scan_interval_m")
    scan_coverage_radius_m = LaunchConfiguration("scan_coverage_radius_m")
    fail_first_n_scans = LaunchConfiguration("fail_first_n_scans")
    mock_scan_duration_s = LaunchConfiguration("mock_scan_duration_s")
    mock_download_duration_s = LaunchConfiguration("mock_download_duration_s")
    device_address = LaunchConfiguration("device_address")
    output_dir = LaunchConfiguration("output_dir")

    stop_scan_params = str(Path(stop_scan_share) / "config" / "stop_scan.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_mock_scanner", default_value="true"),
        DeclareLaunchArgument("scan_interval_m", default_value="2.0"),
        DeclareLaunchArgument("scan_coverage_radius_m", default_value="4.0"),
        DeclareLaunchArgument("fail_first_n_scans", default_value="0"),
        DeclareLaunchArgument("mock_scan_duration_s", default_value="4.0"),
        DeclareLaunchArgument("mock_download_duration_s", default_value="10.0"),
        DeclareLaunchArgument("device_address", default_value="192.168.10.90:8081"),
        DeclareLaunchArgument("output_dir", default_value="scans"),
        # exploration_monitor: stall-based early stop (no ground-truth needed).
        DeclareLaunchArgument("auto_stop_on_stall", default_value="true"),
        DeclareLaunchArgument("stall_timeout_s", default_value="300.0"),
        DeclareLaunchArgument("min_progress_cells", default_value="80"),
        DeclareLaunchArgument("frontier_suppression_enabled", default_value="true"),

        # --- Mapping / navigation / exploration ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "exploration.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "frontier_suppression_enabled":
                    LaunchConfiguration("frontier_suppression_enabled"),
                "use_rviz": use_rviz,
            }.items(),
        ),

        # --- BLK360 scanner: mock (sim) or real (hardware) ---
        GroupAction(
            condition=IfCondition(use_mock_scanner),
            actions=[
                Node(
                    package="blk360_stop_scan",
                    executable="mock_blk360_scanner",
                    name="mock_blk360_scanner",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "scan_duration_s": mock_scan_duration_s,
                        "download_duration_s": mock_download_duration_s,
                        "fail_first_n_scans": fail_first_n_scans,
                    }],
                ),
            ],
        ),
        GroupAction(
            condition=UnlessCondition(use_mock_scanner),
            actions=[
                Node(
                    package="blk360_scanner",
                    executable="scan_node",
                    name="blk360_scanner",
                    output="screen",
                    parameters=[{
                        "device_address": device_address,
                        "output_dir": output_dir,
                    }],
                ),
            ],
        ),

        # --- Stop-scan sequencer ---
        Node(
            package="blk360_stop_scan",
            executable="stop_scan_sequencer",
            name="blk360_stop_scan_sequencer",
            output="screen",
            parameters=[
                stop_scan_params,
                {
                    "use_sim_time": use_sim_time,
                    "scan_interval_m": scan_interval_m,
                    "scan_coverage_radius_m": scan_coverage_radius_m,
                },
            ],
        ),

        # --- Exploration progress monitor + stall-based early stop ---
        Node(
            package="blk360_stop_scan",
            executable="exploration_monitor",
            name="exploration_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "auto_stop": LaunchConfiguration("auto_stop_on_stall"),
                "stall_timeout_s": LaunchConfiguration("stall_timeout_s"),
                "min_progress_cells": LaunchConfiguration("min_progress_cells"),
            }],
        ),
    ])
