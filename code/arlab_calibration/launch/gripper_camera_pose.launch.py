from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
