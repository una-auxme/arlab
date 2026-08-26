from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    zirbi_display = Node(
        package="arlab_ui",
        executable="zirbi_display",
        name="zirbi_display",
        output="screen",
    )

    return LaunchDescription(
        [
            zirbi_display,
        ]
    )
