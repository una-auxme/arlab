"""
Launch the full manipulator simulation stack in the small_house world:
Gazebo + ros2_control + MoveIt/RViz.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manipulator_pkg_share = FindPackageShare("manipulator_description")
    house_pkg_share = FindPackageShare("small_house")
    house_models_path = PathJoinSubstitution([house_pkg_share, "models"])
    default_house_world = PathJoinSubstitution([house_pkg_share, "worlds", "small_house.world"])

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
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start Gazebo with GUI.",
            ),
            DeclareLaunchArgument(
                "launch_rviz_moveit",
                default_value="true",
                description="Start MoveIt RViz for planning and execution.",
            ),
            DeclareLaunchArgument(
                "initial_joint_controller",
                default_value="scaled_joint_trajectory_controller",
                description="Arm controller MoveIt should execute through.",
            ),
            DeclareLaunchArgument(
                "activate_joint_controller",
                default_value="true",
                description="Activate the arm trajectory controller.",
            ),
            DeclareLaunchArgument(
                "launch_servo",
                default_value="false",
                description="Start MoveIt Servo.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo simulation time for MoveIt/RViz.",
            ),
            # Gazebo Sim / ros_gz_sim uses GZ_SIM_RESOURCE_PATH.  The legacy
            # IGN_GAZEBO_RESOURCE_PATH is set as well so the launch file also
            # works on installations that still use the older environment name.
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    house_models_path,
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    house_models_path,
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [manipulator_pkg_share, "launch", "manipulator.full.sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "ur_type": LaunchConfiguration("ur_type"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz_moveit": LaunchConfiguration("launch_rviz_moveit"),
                    "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
                    "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
                    "launch_servo": LaunchConfiguration("launch_servo"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
