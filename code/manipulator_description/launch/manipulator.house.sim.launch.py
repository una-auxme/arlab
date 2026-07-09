"""
Launch the manipulator in the small_house Gazebo world.

This wrapper keeps manipulator.control.sim.launch.py reusable and only adds the
Gazebo resource paths that are needed so model://... entries from the house
world can be resolved after colcon install.
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
                "launch_rviz",
                default_value="false",
                description="Start RViz in addition to Gazebo.",
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
                        [manipulator_pkg_share, "launch", "manipulator.control.sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "ur_type": LaunchConfiguration("ur_type"),
                    "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                    "launch_rviz": LaunchConfiguration("launch_rviz"),
                    "use_mock_hardware": "true",
                }.items(),
            ),
        ]
    )
