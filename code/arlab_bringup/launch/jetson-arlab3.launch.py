from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
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
