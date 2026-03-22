"""custom_spawn_turtlebot3.launch.py

This launch file spawns a TurtleBot3 robot in Gazebo at a specified position and sets up the necessary ROS-Gazebo
bridges for communication. It allows for customization of the robot's initial position through launch arguments.

Author: Jonas Platzer

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for spawning a TurtleBot3 in Gazebo with ROS-Gazebo bridges."""

    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]
    model_folder = "turtlebot3_" + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        model_folder,
        "model.sdf",
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument("x_pose", default_value="0.0", description="Specify namespace of the robot")

    declare_y_position_cmd = DeclareLaunchArgument("y_pose", default_value="0.0", description="Specify namespace of the robot")

    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            TURTLEBOT3_MODEL,
            "-file",
            urdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.01",
        ],
        output="screen",
    )

    bridge_params = "/workspace/build/arlab_movement/params/bridge_params.yaml"

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_image_bridge_cmd) if TURTLEBOT3_MODEL != "burger" else None

    return ld
