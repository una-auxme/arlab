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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """
    ld = LaunchDescription()

    simulation_launch = PathJoinSubstitution([
        FindPackageShare('ur_simulation_gz'),
        'launch',
        'ur_sim_moveit.launch.py'
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="false"))

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        # Falls die Child-Launch Argumente erwartet, hier durchreichen:
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items()
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

    # Video node from perception
    Perception = Node(package="arlab_perception", executable="video_node")

    # Video node from perception
    KnowledgeBase = Node(package="arlab_knowledge", executable="database_node")

    # Video node from perception
    CompVision = Node(package="arlab_computer_vision", executable="object_detection")

    # Add nodes to the launch description
    ld.add_action(sim_include)
    ld.add_action(GetParameter)
    ld.add_action(Orchestrator)
    ld.add_action(Manipulation_CPP)
    #ld.add_action(Perception)
    ld.add_action(KnowledgeBase)
    #ld.add_action(CompVision)

    return ld
