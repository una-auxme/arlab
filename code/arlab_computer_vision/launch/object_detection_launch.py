"""launch.py

Launch file for the ARLab Computer Vision object detection node.

This launch file starts the object_detection node with topic remapping
from camera_color_image to /camera/color/image_raw.

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

    # Declare launch arguments
    enable_continuous_arg = DeclareLaunchArgument(
        "enable_continuous",
        default_value="true",
        description="Enable continuous processing (false = snapshot-only mode)",
    )
    ld.add_action(enable_continuous_arg)

    # Object detection node with topic remapping
    object_detection_node = Node(
        package="arlab_computer_vision",
        executable="object_detection",
        remappings=[
            ("camera_color_image", "/camera/color/image_raw"),
            ("camera_info", "/camera/color/camera_info"),
            ("camera_depth_image", "/camera/depth/image_rect_raw"),
        ],
        parameters=[
            {"log_level": "DEBUG"},  # Node-Parameter für Debug-Logs
            {"enable_continuous": LaunchConfiguration("enable_continuous")},
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
