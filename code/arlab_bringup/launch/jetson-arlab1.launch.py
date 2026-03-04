from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """
    ld = LaunchDescription()

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

    # Add nodes to the launch description
    ld.add_action(robot_control_include)

    return ld
