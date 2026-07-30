"""AMMR proxy in Gazebo Harmonic for the semantic mission stack.

Spawns the ammr model (real footprint + lidar geometry, kinematic
VelocityControl drive) into an arbitrary world and provides the same ROS
contract as the TB3 bringup: /scan, /odom, /cmd_vel (TwistStamped), /clock,
odom->base_footprint TF (bridge) and URDF static TFs (robot_state_publisher).

  ros2 launch blk360_bringup ammr_sim.launch.py world:=<path.world> \
      x_pose:=-2.0 y_pose:=0.5
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("blk360_bringup")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    urdf = os.path.join(
        get_package_share_directory("ammr_description")
        if _has_pkg("ammr_description") else
        "/home/caselab/blk360_ros2_ws/src/ammr_description",
        "urdf", "ammr.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")

    gz_sim_launch = os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r -s -v2 ", world],
                          "on_exit_shutdown": "true"}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=IfCondition(gui),
        launch_arguments={"gz_args": "-g -v2 ",
                          "on_exit_shutdown": "true"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-file", os.path.join(share, "models", "ammr",
                                         "model.sdf"),
                   "-name", "ammr",
                   "-x", LaunchConfiguration("x_pose"),
                   "-y", LaunchConfiguration("y_pose"),
                   "-z", "0.01"])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": os.path.join(share, "config",
                                                 "ammr_bridge.yaml"),
                     "use_sim_time": use_sim_time}])

    camera_bridge = Node(
        package="ros_gz_image", executable="image_bridge",
        output="screen", arguments=["/camera/image_raw"],
        parameters=[{"use_sim_time": use_sim_time}])

    with open(urdf) as f:
        robot_description = f.read()
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("world", description="Full path to .world"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="0.5"),
        gzserver, gzclient, spawn, bridge, camera_bridge, rsp,
    ])


def _has_pkg(name):
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False
