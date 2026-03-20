#!/usr/bin/env python3
# arlab_movement/launch/arlab_simulation_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Pfade zu den relevanten Paketen und Dateien holen
    arlab_movement_pkg_share = get_package_share_directory("arlab_movement")
    nav2_bringup_pkg_share = get_package_share_directory("nav2_bringup")
    gz_sim_pkg_share = get_package_share_directory("ros_gz_sim")

    # --- 1. Launch-Argumente deklarieren ---
    # Diese Argumente machen den Start flexibel.
    world_path = os.path.join(arlab_movement_pkg_share, "worlds", "husarion_office_without_chairs.sdf")
    robot_xacro_path = os.path.join(arlab_movement_pkg_share, "urdf", "turtlebot3_waffle.urdf")
    rviz_config_path = os.path.join(arlab_movement_pkg_share, "rviz", "nav2_slam.rviz")
    nav2_params_path = os.path.join(arlab_movement_pkg_share, "params", "nav2_params.yaml")

    declare_world_cmd = DeclareLaunchArgument("world", default_value=world_path, description="Full path to the world file to load")

    declare_robot_xacro_cmd = DeclareLaunchArgument(
        "robot_xacro", default_value=robot_xacro_path, description="Full path to the robot xacro file"
    )

    declare_headless_cmd = DeclareLaunchArgument("headless", default_value="False", description="Run Gazebo in headless mode (no GUI)")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", default_value=nav2_params_path, description="Full path to the NAV2 parameters file."
    )

    # --- 2. Gazebo Simulation starten ---
    # Startet den Gazebo Server und (optional) den Client (GUI)
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_pkg_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v4 ", LaunchConfiguration("world")]}.items(),
    )

    gz_sim_client = ExecuteProcess(
        cmd=["gz", "sim", "-g"],
        output="screen",
        # Starte die GUI nur, wenn headless:=false gesetzt ist
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("headless"), "'.lower() == 'false'"])),
    )

    # --- 3. Roboter-Beschreibung laden und spawnen ---
    # Das XACRO-File wird in eine URDF-Beschreibung umgewandelt
    robot_description = Command(["xacro ", LaunchConfiguration("robot_xacro")])

    # Ein Knoten, der die Roboterbeschreibung liest und als TF-Frames publiziert
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    # Der Knoten, der den Roboter in Gazebo an Position (0,0,0) einfügt
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "arlab_robot", "-x", "0.0", "-y", "0.0", "-z", "0.1"],
        output="screen",
    )

    # --- 4. SLAM (Nav2) starten ---
    # Wir binden den SLAM-Launch-File von Nav2 ein.
    # Dieser startet die slam_toolbox und alles Notwendige für die Kartenerstellung.
    nav2_slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_pkg_share, "launch", "slam_launch.py")),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": LaunchConfiguration("params_file"),
            "autostart": "True",
        }.items(),
    )

    # --- 5. RViz für die Visualisierung starten ---
    rviz_node = Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz_config_path], output="screen")

    # --- 6. Deine eigenen Knoten starten ---
    # Deine bestehenden Knoten werden hier einfach hinzugefügt.
    arlab_test_node = Node(package="arlab_movement", executable="arlab_movement_test", name="arlab_movement_test", output="screen")

    arlab_orchestrator_node = Node(
        package="arlab_movement", executable="arlab_movement_orchestrator", name="arlab_movement_orchestrator", output="screen"
    )

    # gives the functionality to steer the robot with wasd
    teleop_keyboard_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",  # start node in seperate terminal
    )

    # --- 7. Die LaunchDescription zusammenstellen ---
    # Alle Aktionen werden hier gesammelt und zurückgegeben.
    ld = LaunchDescription()

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_xacro_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(gz_sim_server)
    ld.add_action(gz_sim_client)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_robot_node)
    ld.add_action(nav2_slam_cmd)
    ld.add_action(rviz_node)

    # Deine Knoten hinzufügen
    ld.add_action(arlab_test_node)
    ld.add_action(arlab_orchestrator_node)
    ld.add_action(teleop_keyboard_node)

    return ld
