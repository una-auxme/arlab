"""launch.py

Launch file for the ARLab Computer Vision `ObjectDetection` node.

This launch file starts the node configured for the integrated workflow,
including topic remapping under the `/camera_gripper/...` namespace.

Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
    Peter Viechter <peter.viechter@uni-a.de>
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a ROS2 launch description for the object detection node.

    Returns:
        LaunchDescription: LaunchDescription object containing the
            object_detection node.
    """
    ld = LaunchDescription()

    # Object detection node with topic remapping
    object_detection_node = Node(
        package="arlab_computer_vision",
        executable="object_detection",
        remappings=[
            ("camera_color_image", "/camera_gripper/color/image_raw"),
            ("camera_info", "/camera_gripper/color/camera_info"),
            ("camera_point_cloud", "/camera_gripper/depth/color/points"),
        ],
        parameters=[
            {"log_level": "INFO"},  # Node parameter for logging verbosity
            {"sync_tolerance": 5.0},
            {"target_frame": "world"},
            {"snapshot_mode": False},  # SOLUTION FOR TASK a) Disable snapshot mode
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "ObjectDetection:=info",  # Set only this node log level to INFO
            "--log-level",
            "rcl:=warn",  # ROS 2 internal logs at WARN
            "--log-level",
            "rclpy:=warn",  # ROS 2 Python logs at WARN
        ],
    )

    # Add node to the launch description
    ld.add_action(object_detection_node)

    return ld
