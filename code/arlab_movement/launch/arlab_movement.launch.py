from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arlab_movement',
            namespace='arlab_movement',
            executable='arlab_movement_test',
            name='arlab_movement_test',
            output='screen'
        ),
    ])