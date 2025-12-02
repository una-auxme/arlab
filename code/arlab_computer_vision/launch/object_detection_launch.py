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
            ("camera_color_image", "/color/image_raw"),
            ("camera_info", "/color/camera_info"),
            ("camera_depth_image", "/depth/image_rect_raw"),
        ],
    )

    # Add node to the launch description
    ld.add_action(object_detection_node)

    return ld
