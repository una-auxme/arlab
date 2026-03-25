"""
Launch file for testing the ARLab Manipulation orchestrator system.

This launch file starts all relevant ROS2 nodes for the manipulation pipeline,
including service nodes, Python orchestrator, and C++ subscriber nodes.

Maintainer:
    Sofia Öttl <sofia.oettl@uni-a.de>
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Nodes started:
        1. GetGrippingParameter service node
           - Provides gripping force, grip modes, and object weight for manipulator.
        2. Orchestrator Python node
           - Handles action requests, queries knowledge, computes grasp/placement.
        3. OrchestratorSubscriber C++ node
           - Subscribes to orchestrator data for downstream MoveIt or robot control.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """
    ld = LaunchDescription()

    # Service node for providing gripping force recommendations
    GetParameter = Node(
        package="arlab_manipulation",
        executable="gripping_parameter",
    )

    # Python orchestrator node that handles perception and planning
    Orchestrator = Node(package="arlab_manipulation", executable="orchestrator")

    # C++ subscriber node for orchestrator data (for MoveIt or robot control)
    Manipulation_CPP = Node(package="arlab_manipulation_cpp", executable="Manipulation_CPP")

    # Add nodes to the launch description
    ld.add_action(GetParameter)
    ld.add_action(Orchestrator)
    ld.add_action(Manipulation_CPP)

    return ld
