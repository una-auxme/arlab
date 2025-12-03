#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package directory
    package_name = 'arlab_movement'  
    package_dir = FindPackageShare(package=package_name)
    
    # Path to parameter file
    param_file = PathJoinSubstitution([
        package_dir, 'params', 'arlab_navigation_params.yaml'
    ])
    
    # Navigation Stack Manager Node
    navigation_manager = Node(
        package='arlab_movement',
        executable='navigation_stack_manager',
        name='navigation_stack_manager',
        output='screen',
        parameters=[param_file]
    )
    
    # Map Save Publisher Node
    map_save_publisher = Node(
        package='arlab_movement',  
        executable='map_save_publisher',
        name='map_save_publisher',
        output='screen',
        parameters=[param_file]
    )
    
    return LaunchDescription([
        navigation_manager,
        map_save_publisher,
    ])