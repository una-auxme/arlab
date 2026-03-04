"""launch.py

Launch file for testing the ARLab Manipulation orchestrator system.

This launch file starts:
1. GetGrippingForce service node.
2. Orchestrator Python node.
3. OrchestratorSubscriber C++ node.
4. ...

Author: Sofia Öttl
Date: 2025-11-02

"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """
    ld = LaunchDescription()

    manipulator_launch = PathJoinSubstitution(
        [
            FindPackageShare("manipulator_description"),
            "launch",
            "manipulator.launch.py",
        ]
    )

    manipulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manipulator_launch),
        # Falls die Child-Launch Argumente erwartet, hier durchreichen:
        launch_arguments={}.items(),
    )

    # Service node for providing gripping force recommendations
    GetParameter = Node(
        package="arlab_manipulation",
        executable="gripping_parameter",
    )

    # Python orchestrator node that handles perception and planning
    Orchestrator = Node(package="arlab_manipulation", executable="orchestrator")

    # C++ subscriber node for orchestrator data (for MoveIt or robot control)
    Manipulation_CPP = Node(
        package="arlab_manipulation_cpp", executable="Manipulation_CPP"
    )

    # Add nodes to the launch description
    ld.add_action(manipulator_include)
    ld.add_action(GetParameter)
    ld.add_action(Orchestrator)
    ld.add_action(Manipulation_CPP)

    return ld
