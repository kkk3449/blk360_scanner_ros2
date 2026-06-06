"""Top-level active-mapping bringup: Cartographer 2D SLAM + Nav2 + frontier explorer.

This launches ONLY the mapping/navigation/exploration stack. The robot itself
(TurtleBot3 simulation or real hardware publishing /scan, /odom, and the
odom->base_footprint TF) must be started separately, e.g.:

  ros2 launch blk360_bringup tb3_sim.launch.py        # simulation
  # or your real robot bringup

Then:

  ros2 launch blk360_bringup exploration.launch.py use_sim_time:=true
"""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("blk360_bringup")
    launch_dir = Path(bringup_share) / "launch"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    frontier_params = LaunchConfiguration("frontier_params")
    nav2_params = LaunchConfiguration("nav2_params")
    # Override of frontier_suppression_enabled (for ablation); empty -> use the
    # value from frontier_params.yaml.
    suppression = LaunchConfiguration("frontier_suppression_enabled")

    default_frontier_params = str(Path(bringup_share) / "config" / "frontier" / "frontier_params.yaml")
    default_nav2_params = str(Path(bringup_share) / "config" / "nav2" / "nav2_params.yaml")
    rviz_config = str(Path(bringup_share) / "rviz" / "exploration.rviz")

    cartographer_launch = str(launch_dir / "cartographer.launch.py")
    nav2_launch = str(launch_dir / "nav2.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("frontier_params", default_value=default_frontier_params),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        # Matches the yaml default; ablation flips it to compare suppression on/off.
        DeclareLaunchArgument("frontier_suppression_enabled", default_value="true"),

        # Cartographer 2D SLAM -> /map + map->odom TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # Nav2 (SLAM mode) -> navigate_to_pose + costmaps
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
            }.items(),
        ),

        # Frontier exploration -> dispatches navigate_to_pose goals.
        # Launched as a plain Node (not the package's own launch file, whose
        # parameter-override dict passes typed params as strings and aborts).
        Node(
            package="frontier_exploration_ros2",
            executable="frontier_explorer",
            name="frontier_explorer",
            output="screen",
            parameters=[
                frontier_params,
                {"use_sim_time": use_sim_time,
                 "frontier_suppression_enabled": ParameterValue(suppression, value_type=bool)},
            ],
        ),

        GroupAction(
            condition=IfCondition(use_rviz),
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="screen",
                ),
            ],
        ),
    ])
