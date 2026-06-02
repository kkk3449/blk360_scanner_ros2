"""TurtleBot3 Gazebo (Harmonic) simulation bringup for active-mapping tests.

Starts the Gazebo server (always) and client (only when gui:=true, so the stack
can be verified headless), plus robot_state_publisher and the TurtleBot3 spawn.
Provides /scan, /odom, /cmd_vel and the odom->base_footprint TF that the mapping
stack consumes.

  ros2 launch blk360_bringup tb3_sim.launch.py                 # headless
  ros2 launch blk360_bringup tb3_sim.launch.py gui:=true       # with Gazebo GUI
  TURTLEBOT3_MODEL=waffle ros2 launch blk360_bringup tb3_sim.launch.py world:=house
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    tb3_launch_dir = os.path.join(tb3_gazebo, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    model = LaunchConfiguration("model")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    # `world` is a full path to an SDF .world file (defaults to turtlebot3_world).
    world = LaunchConfiguration("world")

    gz_sim_launch = os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s -v2 ", world],
                          "on_exit_shutdown": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=IfCondition(gui),
        launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "true"}.items(),
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("model", default_value="waffle",
                              description="TurtleBot3 model: burger | waffle | waffle_pi"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(tb3_gazebo, "worlds", "turtlebot3_world.world"),
            description="Full path to an SDF .world file (default: turtlebot3_world)"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),

        SetEnvironmentVariable("TURTLEBOT3_MODEL", model),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                                  os.path.join(tb3_gazebo, "models")),

        gzserver,
        gzclient,
        robot_state_publisher,
        spawn,
    ])
