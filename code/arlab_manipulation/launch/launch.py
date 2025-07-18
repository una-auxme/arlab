from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    GetGrippingForce = Node(
        package="arlab_manipulation",
        executable="GetGrippingForce",
    )

    Orchestrator = Node(
        package="arlab_manipulation",
        executable="Orchestrator"
    )

    PoseSubscriber = Node(
        package="arlab_manipulation_cpp",
        executable="PoseSubscriber"
    )

    ld.add_action(GetGrippingForce)
    ld.add_action(Orchestrator)
    ld.add_action(PoseSubscriber)

    return ld