"""
Launch file: manipulator.control.robot.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Starts the low-level hardware drivers for the real robot:
    - UR arm driver  (manipulator_robot_driver/ur_control.launch.py)
    - Mia Hand driver (mia_hand_driver/mia_hand_driver_launch.py)

Use this launch file if you only need joint-level control without MoveIt.
For the full stack including MoveIt use manipulator.full.robot.launch.py.
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
    """Resolve launch arguments and build the list of actions to start.

    Args:
        context: Launch context used to resolve substitutions.
        *args: Unused positional arguments required by OpaqueFunction.
        **kwargs: Unused keyword arguments required by OpaqueFunction.

    Returns:
        List of launch actions to execute.
    """
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    serial_port_arg = LaunchConfiguration("serial_port_arg")

    # Launch the UR arm driver with ros2_control and the joint trajectory controller.
    manipulator_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_robot_driver"),
                    "launch",
                    "ur_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": launch_rviz,
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "mock_sensor_commands": mock_sensor_commands,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
        }.items(),
    )

    # Launch the Mia Hand serial driver.
    mia_hand_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mia_hand_driver"),
                    "launch",
                    "mia_hand_driver_launch.py",
                ]
            )
        ),
        launch_arguments={
            "serial_port": serial_port_arg,
        }.items(),
    )

    return [
        manipulator_driver_launch,
        mia_hand_driver_launch,
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
            description="Enable mock command interfaces for sensors used for simple simulations. "
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
