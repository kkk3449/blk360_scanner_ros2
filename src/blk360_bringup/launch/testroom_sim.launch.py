"""TurtleBot3 in the custom testroom world (built from testroom260601.e57).

Thin wrapper over tb3_sim.launch.py that points Gazebo at the generated
testroom.world and spawns the robot at a collision-free point inside the room.

  ros2 launch blk360_bringup testroom_sim.launch.py gui:=true
then:
  ros2 launch blk360_bringup active_mapping.launch.py use_sim_time:=true
"""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("blk360_bringup")
    world = str(Path(share) / "worlds" / "testroom.world")
    tb3_sim = str(Path(share) / "launch" / "tb3_sim.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("model", default_value="waffle"),
        # Collision-free spawn computed by occ_to_world.py for this map.
        DeclareLaunchArgument("x_pose", default_value="1.83"),
        DeclareLaunchArgument("y_pose", default_value="-0.62"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_sim),
            launch_arguments={
                "use_sim_time": "true",
                "gui": LaunchConfiguration("gui"),
                "model": LaunchConfiguration("model"),
                "world": world,
                "x_pose": LaunchConfiguration("x_pose"),
                "y_pose": LaunchConfiguration("y_pose"),
            }.items(),
        ),
    ])
