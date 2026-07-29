from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BLK = "/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gated", default_value="true"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        Node(
            package="semantic_nav_bt",
            executable="mission_bt",
            output="screen",
            parameters=[{
                "kg_path": f"{BLK}/testroom_epochs_kg.json",
                "places_path": f"{BLK}/place_layer_T3_slic.json",
                "naming_path": f"{BLK}/place_ring_naming.json",
                "map_yaml": "/home/caselab/ammr_twin/map_vis_n2_1.yaml",
                "gated": LaunchConfiguration("gated"),
                "dry_run": LaunchConfiguration("dry_run"),
            }],
        ),
    ])
