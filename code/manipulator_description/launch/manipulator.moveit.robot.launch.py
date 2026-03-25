"""
Launch file: manipulator.moveit.robot.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Starts MoveIt move_group and optionally RViz for the real robot.
Delegates entirely to manipulator_ur_moveit_config/ur_moveit.launch.py.

This file is typically not called directly — it is included by
manipulator.full.robot.launch.py. Call it standalone only if the hardware
drivers are already running in a separate terminal.
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
    """Resolve launch arguments and forward them to the MoveIt config launch file.

    Args:
        context: Launch context used to resolve substitutions.
        *args: Unused positional arguments required by OpaqueFunction.
        **kwargs: Unused keyword arguments required by OpaqueFunction.

    Returns:
        List containing the MoveIt launch action.
    """
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    ur_type = LaunchConfiguration("ur_type")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # Forward all arguments to the MoveIt config package launch file.
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
            "launch_rviz_moveit": launch_rviz_moveit,
            "ur_type": ur_type,
            "launch_servo": launch_servo,
            "use_sim_time": use_sim_time,
            "publish_robot_description_semantic": publish_robot_description_semantic,
        }.items(),
    )

    return [
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
