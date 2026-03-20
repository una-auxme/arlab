"""Robot control and gripper camera calibration launch file

Launch configuration:
    Includes the robot control launch file from `manipulator_description`
    Includes the gripper camera calibration launch file from `arlab_calibration`
    Starts the robot control and calibration-related nodes

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for robot control and calibration running on Jetson 1.

    Returns:
        LaunchDescription: Launch description that includes the robot control launch
        configuration and the gripper camera calibration.
    """

    robot_control_launch = PathJoinSubstitution(
        [
            FindPackageShare("manipulator_description"),
            "launch",
            "manipulator.control.robot.launch.py",
        ]
    )

    robot_control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_control_launch),
        launch_arguments={}.items(),
    )

    gripper_camera_calibration_launch = PathJoinSubstitution(
        [
            FindPackageShare("arlab_calibration"),
            "launch",
            "gripper_camera_pose.launch.py",
        ]
    )

    gripper_camera_calibration_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gripper_camera_calibration_launch),
        launch_arguments={}.items(),
    )

    return LaunchDescription([robot_control_include, gripper_camera_calibration_include])
