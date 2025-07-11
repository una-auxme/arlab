from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    GetGrippingForce = Node(
        package="arlab_manipulation",
        executable="GetGrippingForce",
    )

    PosePublisher = Node(
        package="arlab_manipulation",
        executable="PosePublisher"
    )

    PoseSubscriber = Node(
        package="arlab_manipulation_cpp",
        executable="PoseSubscriber"
    )

    ld.add_action(GetGrippingForce)
    ld.add_action(PosePublisher)
    ld.add_action(PoseSubscriber)

    return ld