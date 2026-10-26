from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ros_gz_bridge_launch_file = os.path.join(
        get_package_share_directory('ros_gz_bridge'),
        'launch',
        'ros_gz_bridge.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_gz_bridge_launch_file),
            launch_arguments={
                'bridge_name': 'camera_gripper_bridge',
                'config_file': '/workspace/src/arlab/code/manipulator_description/config/camera_gripper_bridge.yaml',
            }.items()
        )
    ])