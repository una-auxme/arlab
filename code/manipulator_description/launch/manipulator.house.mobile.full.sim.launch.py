"""
Launch the mobile manipulator in the small_house world.

This variant uses manipulator_mobile.urdf.xacro:
  - Ranger Air base is not fixed to world
  - Gazebo VelocityControl subscribes to /cmd_vel for omnidirectional base motion
  - Gazebo OdometryPublisher publishes /odom and odom -> ranger_base_link
  - a simulated 2D lidar publishes /scan for SLAM/Nav2
  - MoveIt/RViz still controls the UR arm through ros2_control
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manipulator_pkg_share = FindPackageShare("manipulator_description")
    house_pkg_share = FindPackageShare("small_house")
    house_models_path = PathJoinSubstitution([house_pkg_share, "models"])
    default_house_world = PathJoinSubstitution([house_pkg_share, "worlds", "small_house.world"])
    mobile_description = PathJoinSubstitution(
        [manipulator_pkg_share, "urdf", "manipulator_mobile.urdf.xacro"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_file",
                default_value=default_house_world,
                description="World file to load. Defaults to the bundled small_house world.",
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="UR type to spawn, e.g. ur5e or ur10e.",
            ),
            DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start Gazebo GUI."),
            DeclareLaunchArgument(
                "launch_rviz_moveit",
                default_value="true",
                description="Start MoveIt RViz for arm planning and execution.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use Gazebo simulation time."),
            DeclareLaunchArgument("spawn_x", default_value="1.0", description="Initial mobile base x position."),
            DeclareLaunchArgument("spawn_y", default_value="1.0", description="Initial mobile base y position."),
            DeclareLaunchArgument("spawn_z", default_value="0.0", description="Initial mobile base z position."),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Initial mobile base yaw."),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[house_models_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[house_models_path, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([manipulator_pkg_share, "launch", "manipulator.full.sim.launch.py"])
                ),
                launch_arguments={
                    "description_file": mobile_description,
                    "world_file": LaunchConfiguration("world_file"),
                    "ur_type": LaunchConfiguration("ur_type"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz_moveit": LaunchConfiguration("launch_rviz_moveit"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                    "activate_joint_controller": "true",
                    "spawn_x": LaunchConfiguration("spawn_x"),
                    "spawn_y": LaunchConfiguration("spawn_y"),
                    "spawn_z": LaunchConfiguration("spawn_z"),
                    "spawn_yaw": LaunchConfiguration("spawn_yaw"),
                }.items(),
            ),
            # Gazebo transport <-> ROS 2 bridges for the mobile base and lidar.
            # The arm controller itself is still handled by gz_ros2_control.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mobile_base_gz_bridge",
                output="screen",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "--ros-args",
                    "--log-level",
                    "error",
                ],
            ),
        ]
    )
