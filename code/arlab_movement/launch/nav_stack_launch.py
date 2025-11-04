from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindPackageShare
import os

def generate_launch_description():
    bringup_dir = FindPackageShare("nav2_bringup").find("nav2_bringup")
    launch_file_dir = os.path.join(bringup_dir, "launch")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, "bringup_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "map": os.path.join(
                    FindPackageShare("arlab_movement").find("arlab_movement"),
                    "maps",
                    "husarion_world.yaml"
                ),
                "params_file": os.path.join(
                    FindPackageShare("arlab_movement").find("arlab_movement"),
                    "params",
                    "nav2_params.yaml"
                )
            }.items(),
        ),
    ])
