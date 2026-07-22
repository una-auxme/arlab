from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    #import the yaml parameters
    config = os.path.join(
        get_package_share_directory("arlab_whisper"),
        "config",
        "whisper.yaml",
    )

    return LaunchDescription([
        Node(
            package="arlab_whisper",
            executable="whisper",
            name="whisper",
            output="screen",
            parameters=[config],
        ),
    ])