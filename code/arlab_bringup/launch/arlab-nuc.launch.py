"""Orchestrator launch file

Launch configuration:
    Includes the MoveIt robot launch file from `manipulator_description`
    Includes the manipulation launch file from `arlab_manipulation`
    Starts the required nodes for robot motion planning and manipulation

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a ROS2 launch description for nodes running on the NUC.

    Returns:
        LaunchDescription: Launch description that includes the robot MoveIt launch
        configuration and the manipulation launch file.
    """
    ld = LaunchDescription()

    robot_moveit_launch = PathJoinSubstitution(
        [
            FindPackageShare("manipulator_description"),
            "launch",
            "manipulator.moveit.robot.launch.py",
        ]
    )

    robot_moveit_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_moveit_launch),
        launch_arguments={}.items(),
    )

    manipulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("arlab_manipulation"),
            "launch",
            "launch.py",
        ]
    )

    manipulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manipulation_launch),
        launch_arguments={}.items(),
    )

    # Add nodes to the launch description
    ld.add_action(robot_moveit_include)
    ld.add_action(manipulation_include)

    return ld
