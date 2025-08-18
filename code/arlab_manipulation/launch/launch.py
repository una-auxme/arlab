# Copyright 2025 Sofia Öttl, University of Augsburg
#
# License????

"""launch.py

Launch file for the ARLab Manipulation orchestrator system.

This launch file starts:
1. GetGrippingForce service node.
2. Orchestrator Python node.
3. OrchestratorSubscriber C++ node.

"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """
    ld = LaunchDescription()

    # Service node for providing gripping force recommendations 
    GetGrippingForce = Node(
        package="arlab_manipulation",
        executable="GetGrippingForce",
    )

    # Python orchestrator node that handles perception and planning
    Orchestrator = Node(
        package="arlab_manipulation",
        executable="Orchestrator"
    )

    # C++ subscriber node for orchestrator data (for MoveIt or robot control)
    OrchestratorSubscriber = Node(
        package="arlab_manipulation_cpp",
        executable="OrchestratorSubscriber"
    )

    # Add nodes to the launch description
    ld.add_action(GetGrippingForce)
    ld.add_action(Orchestrator)
    ld.add_action(OrchestratorSubscriber)

    return ld
