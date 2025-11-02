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
        executable="parameter_service",
    )

    # Python orchestrator node that handles perception and planning
    Orchestrator = Node(package="arlab_manipulation", executable="orchestrator")

    # C++ subscriber node for orchestrator data (for MoveIt or robot control)
    OrchestratorSubscriber = Node(
        package="arlab_manipulation_cpp", executable="OrchestratorSubscriber"
    )

    # Video node from perception
    Perception = Node(package="arlab_perception", executable="video_node")

    # Video node from perception
    KnowledgeBase = Node(package="arlab_knowledge", executable="database_node")

    # Video node from perception
    ComputerVision = Node(package="arlab_computer_vision", executable="object_detection")

    # Add nodes to the launch description
    ld.add_action(GetGrippingForce)
    ld.add_action(Orchestrator)
    ld.add_action(OrchestratorSubscriber)
    ld.add_action(Perception)
    ld.add_action(KnowledgeBase)
    ld.add_action(ComputerVision)

    return ld
