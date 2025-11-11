# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# ...

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the TURTLEBOT3_MODEL environment variable
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL

    # Path to the robot SDF model
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables for robot position
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Launch arguments for position
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial X position of the robot'
    )
    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial Y position of the robot'
    )

    # Launch argument for bridge config file
    declare_bridge_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('arlab_movement'),
            'params',
            'corrected_bridge_turtlebot.yaml'
        ),
        description='Path to the ros_gz_bridge YAML file'
    )

    # Use LaunchConfiguration for bridge config
    bridge_config = LaunchConfiguration('config_file')

    # Node: Spawn the robot in Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    # Node: Start the ROS-Gazebo bridge using YAML
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[bridge_config],  # Correct way to load YAML
        output='screen',
    )

    # Node: Start image bridge if the model is not "burger"
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # Build the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_bridge_config_arg)

    # Add nodes
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    if TURTLEBOT3_MODEL != 'burger':
        ld.add_action(start_gazebo_ros_image_bridge_cmd)

    return ld
