"""
Launch file: manipulator.full.sim.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Top-level launch file for the Gazebo simulation. Composes two sub-stacks:
    1. manipulator.control.sim.launch.py         — Gazebo + ros2_control controllers
    2. manipulator_ur_moveit_config/ur_moveit.launch.py — MoveIt + RViz

This is the recommended entry point for simulation with MoveIt planning.

Based on the original UR simulation launch file by Denis Stogl /
Stogl Robotics Consulting UG.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and compose the simulation and MoveIt sub-stacks.

    Args:
        context: Launch context used to resolve substitutions.
        *args: Unused positional arguments required by OpaqueFunction.
        **kwargs: Unused keyword arguments required by OpaqueFunction.

    Returns:
        List of launch actions to execute.
    """
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    controllers_file = LaunchConfiguration("controllers_file")

    # Start Gazebo with all ros2_control controllers.
    # RViz is disabled here because MoveIt starts its own RViz instance below.
    manipulator_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "launch",
                    "manipulator.control.sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "controllers_file": controllers_file,
            "launch_rviz": "false",
        }.items(),
    )

    # Start MoveIt move_group and RViz with simulation time enabled.
    manipulator_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_ur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        manipulator_sim_control_launch,
        manipulator_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    """Declare all launch arguments and register the OpaqueFunction setup.

    Returns:
        LaunchDescription containing all declared arguments and the setup function.
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([FindPackageShare("manipulator_description"), "config", "manipulator_controllers.yaml"]),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
