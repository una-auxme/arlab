from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    Get_Gripping_Force = Node(
        package="arlab_manipulation",
        executable="Get_Gripping_Force",
    )

    PosePublisher = Node(
        package="arlab_manipulation",
        executable="PosePublisher"
    )

    PoseSubscriber = Node(
        package="arlab_manipulation_cpp",
        executable="PoseSubscriber"
    )

    ld.add_action(Get_Gripping_Force)
    ld.add_action(PosePublisher)
    ld.add_action(PoseSubscriber)

    return ld