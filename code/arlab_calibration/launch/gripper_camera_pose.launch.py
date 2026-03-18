"""Periodic transform publisher launch file

Launch this file with:
    `ros2 launch arlab_calibration <launch_file_name>.py`

Launch configuration:
    Starts the `periodic_transform_publisher` node from `arlab_calibration`
    Publishes a transform from `tool0` to `camera_gripper_link`
    Sets the fixed translation and rotation parameters for the transform

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the periodic transform publisher.

    Creates a launch description that starts the
    `periodic_transform_publisher` node with the configured transform
    parameters between `tool0` and `camera_gripper_link`.
    """
    return LaunchDescription(
        [
            Node(
                package="arlab_calibration",
                executable="periodic_transform_publisher",
                parameters=[
                    {"frame_id": "tool0"},
                    {"child_frame_id": "camera_gripper_link"},
                    {"x": 0.0205783},
                    {"y": 0.118494},
                    {"z": 0.0265386},
                    {"qx": -0.433977},
                    {"qy": -0.433066},
                    {"qz": -0.562931},
                    {"qw": 0.55428},
                ],
            ),
        ]
    )
