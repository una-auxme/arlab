"""
Launch file: manipulator.full.robot.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Top-level launch file for the real robot. Composes two sub-stacks:
    1. manipulator.control.robot.launch.py  — UR arm driver + Mia Hand driver
    2. manipulator.moveit.robot.launch.py   — MoveIt move_group + RViz

This is the recommended entry point for operating the physical robot.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and compose the control and MoveIt sub-stacks.

    Args:
        context: Launch context used to resolve substitutions.
        *args: Unused positional arguments required by OpaqueFunction.
        **kwargs: Unused keyword arguments required by OpaqueFunction.

    Returns:
        List of launch actions to execute.
    """
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    serial_port_arg = LaunchConfiguration("serial_port_arg")

    # Start the hardware drivers for the UR arm and the Mia Hand.
    manipulator_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "launch",
                    "manipulator.control.robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "robot_ip": robot_ip,
            "ur_type": ur_type,
            "serial_port_arg": serial_port_arg,
        }.items(),
    )

    # Start MoveIt move_group and optionally RViz with the MoveIt plugin.
    manipulator_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "launch",
                    "manipulator.moveit.robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz_moveit": launch_rviz_moveit,
        }.items(),
    )

    return [
        manipulator_control_launch,
        manipulator_moveit_launch,
    ]


def generate_launch_description():
    """Declare all launch arguments and register the OpaqueFunction setup.

    Returns:
        LaunchDescription containing all declared arguments and the setup function.
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz_moveit",
            default_value="true",
            description="Launch RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Typo/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Launch servo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="MoveGroup publishes robot description semantic",
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.135.245.20",  # Replace with your robot's IP address.
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors "
            "used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port_arg",
            default_value="/dev/ttyUSB0",
            description="Mia Hand serial port device.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
