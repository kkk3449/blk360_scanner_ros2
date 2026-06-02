"""Launch the BLK360 stop-scan sequencer with its parameter file."""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("blk360_stop_scan")
    default_params = str(Path(pkg_share) / "config" / "stop_scan.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),

        Node(
            package="blk360_stop_scan",
            executable="stop_scan_sequencer",
            name="blk360_stop_scan_sequencer",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
