"""Speech controller launch file

Launch configuration:
    Starts the `moshi_tts` node from `arlab_speech_controller`
    Configures the voice file used for speech synthesis
    Sets the maximum offset parameter for playback

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the speech controller running on Jetson 2.

    Returns:
        LaunchDescription: Launch description that starts the `moshi_tts` node with
        the configured voice file and playback parameters.
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
