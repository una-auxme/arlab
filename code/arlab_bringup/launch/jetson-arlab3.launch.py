"""Knowledge and computer vision launch file

Launch configuration:
    Includes the knowledge system launch file from `arlab_knowledge`
    Includes the object detection launch file from `arlab_computer_vision`
    Starts the knowledge and computer vision related components

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for knowledge and computer vision running on Jetson 3.

    Returns:
        LaunchDescription: Launch description that includes the knowledge launch file
        and the object detection launch file.
    """

    knowledge_launch = PathJoinSubstitution(
        [
            FindPackageShare("arlab_knowledge"),
            "launch",
            "knowledge_launch.py",
        ]
    )

    knowledge_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(knowledge_launch),
        launch_arguments={}.items(),
    )

    computer_vision_launch = PathJoinSubstitution(
        [
            FindPackageShare("arlab_computer_vision"),
            "launch",
            "object_detection_launch.py",
        ]
    )

    computer_vision_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(computer_vision_launch),
        launch_arguments={}.items(),
    )

    return LaunchDescription([knowledge_include, computer_vision_include])
