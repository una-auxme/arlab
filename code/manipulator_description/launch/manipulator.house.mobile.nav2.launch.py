"""Start the mobile manipulator simulation plus SLAM Toolbox and Nav2.

Use this after the basic mobile launch works. Nav2 receives /scan and /odom,
SLAM Toolbox publishes map -> odom, and Nav2 publishes /cmd_vel, which is
bridged back to Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manipulator_pkg_share = FindPackageShare("manipulator_description")
    movement_pkg_share = FindPackageShare("arlab_movement")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz_moveit", default_value="true"),
            DeclareLaunchArgument("launch_nav2", default_value="true"),
            DeclareLaunchArgument("launch_slam", default_value="true"),
            DeclareLaunchArgument("launch_nav2_rviz", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="-4.0"),
            DeclareLaunchArgument("spawn_y", default_value="1.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [manipulator_pkg_share, "launch", "manipulator.house.mobile.full.sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "ur_type": LaunchConfiguration("ur_type"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz_moveit": LaunchConfiguration("launch_rviz_moveit"),
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([movement_pkg_share, "launch", "slam_async.launch.py"])
                ),
                condition=IfCondition(LaunchConfiguration("launch_slam")),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file": PathJoinSubstitution(
                        [movement_pkg_share, "params", "ranger_slam_params.yaml"]
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([movement_pkg_share, "launch", "ranger_nav2.launch.py"])
                ),
                condition=IfCondition(LaunchConfiguration("launch_nav2")),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": PathJoinSubstitution(
                        [movement_pkg_share, "params", "ranger_nav2_params.yaml"]
                    ),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav2",
                output="log",
                arguments=["-d", PathJoinSubstitution([movement_pkg_share, "rviz", "nav2_default_view.rviz"])],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("launch_nav2_rviz")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="front_lidar_scoped_frame_tf",
                output="screen",
                arguments=[
                    "0", "0", "0",
                    "0", "0", "0",
                    "front_lidar_link",
                    "ur5e/ranger_base_link/front_lidar",
                ],
            ),
        ]
    )
