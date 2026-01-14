from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    easy_handeye_dir = get_package_share_directory('easy_handeye2')

    return LaunchDescription([
        # Kalibrierung starten (Eye-on-Base)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(easy_handeye_dir, 'launch', 'calibrate.launch.py')
            ),

            launch_arguments={
                'calibration_type': 'eye_on_base',   # ROS 2 Param
                'name': 'my_eob_calib',             # Name der Kalibrierung
                'robot_base_frame': '/base_link',
                'robot_effector_frame': '/ee_link',
                'tracking_base_frame': '/optical_origin',
                'tracking_marker_frame': '/optical_target',
            }.items()
        ),

        # Eye-on-Base Publish starten
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(easy_handeye_dir, 'launch', 'publish.launch.py')
            ),
            launch_arguments={
                'name': 'my_eob_calib'  # Muss mit Kalibrierungsnamen übereinstimmen
            }.items()
        ),
    ])

