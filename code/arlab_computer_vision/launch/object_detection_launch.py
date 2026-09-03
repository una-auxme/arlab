"""launch.py

Launch file for the ARLab Computer Vision `ObjectDetection` node.

This launch file starts the node configured for the integrated workflow,
including topic remapping under the `/camera_gripper/...` namespace.

Maintainers:
    Aleksander Michalak <aleksander1.michalak@uni-a.de>
    Peter Viechter <peter.viechter@uni-a.de>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a ROS2 launch description for the object detection node.

    Returns:
        LaunchDescription: LaunchDescription object containing the
            object_detection node.
    """
    ld = LaunchDescription()

    declare_target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="world",
        description="TF frame the detected entities are transformed into and stored as.",
    )

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
            {"target_frame": LaunchConfiguration("target_frame")},
            {"snapshot_mode": True},
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

    # Add declared arguments and node to the launch description
    ld.add_action(declare_target_frame)
    ld.add_action(object_detection_node)

    return ld
