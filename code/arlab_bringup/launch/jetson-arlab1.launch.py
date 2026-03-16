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
