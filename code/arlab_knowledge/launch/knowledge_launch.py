from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arlab_knowledge',
            namespace='arlab_knowledge',
            executable='knowledge_visualization',
            name='knowledge_visualization'
        ),
        Node(
            package='arlab_knowledge',
            namespace='arlab_knowledge',
            executable='database_node',
            name='database_node'
        ),
    ])
