"""Nav2 bringup in SLAM mode (no map_server / no amcl).

Includes nav2_bringup's navigation_launch.py with our params. Cartographer
provides /map and the map->odom transform, so only the navigation stack
(controller, planner, behaviors, costmaps, bt_navigator) is started.
"""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("blk360_bringup")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params = str(Path(bringup_share) / "config" / "nav2" / "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    navigation_launch = str(Path(nav2_bringup_share) / "launch" / "navigation_launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": autostart,
                "use_composition": "False",
            }.items(),
        ),
    ])
