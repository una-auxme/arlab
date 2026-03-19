from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a ROS2 launch description for orchestrator nodes.

    Returns:
        LaunchDescription: LaunchDescription object containing all nodes.
    """

    voice_launch = Node(
        package="arlab_speech_controller",
        executable="moshi_tts",
        parameters=[
            {
                "voice": "/workspace/src/arlab/Fast Lars opmitized 9s.wav",
                "max_offset": 400,
            }
        ],
    )

    return LaunchDescription([voice_launch])
