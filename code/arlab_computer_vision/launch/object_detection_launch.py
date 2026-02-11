"""launch.py

Launch file for the ARLab Computer Vision object detection node.

This launch file starts the object_detection node with topic remapping
from camera_color_image to /camera/color/image_raw.

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
            {"log_level": "DEBUG"},  # Node-Parameter für Debug-Logs
            {"sync_tolerance": 1.0},
            {"target_frame": "world"},
            {"snapshot_mode": True},
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            "ObjectDetection:=debug",  # Nur unser Node auf DEBUG
            "--log-level",
            "rcl:=warn",  # ROS2 interne Logs auf WARN
            "--log-level",
            "rclpy:=warn",  # ROS2 Python Logs auf WARN
        ],
    )

    # Add node to the launch description
    ld.add_action(object_detection_node)

    return ld
