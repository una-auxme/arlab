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
