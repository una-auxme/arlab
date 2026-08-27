"""mapping_drive.launch.py

Bring-up for a mapping drive: starts the movement orchestrator and,
by default, automatically triggers the mapping (SLAM) and auto-annotation
behaviours so the operator only has to drive the robot, while system auto
maps and annotates the environment.

Maintainers:
    Luca Kahlenberg <luca.kahlenberg@uni-a.de>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Orchestrator action driven by the auto-start block.
MOVEMENT_ACTION_NAME = "/movement/action"
MOVEMENT_ACTION_TYPE = "arlab_common_interfaces/action/MovementAction"


def _send_movement_goal(cmd: str) -> ExecuteProcess:
    """Build an ExecuteProcess that sends a single MovementAction goal via the ros2 CLI."""
    return ExecuteProcess(
        cmd=[
            "ros2",
            "action",
            "send_goal",
            MOVEMENT_ACTION_NAME,
            MOVEMENT_ACTION_TYPE,
            f"{{cmd: {cmd}, enable: true}}",
        ],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    """Resolve arguments and assemble the orchestrator node and auto-start timers."""
    params_file = LaunchConfiguration("params_file").perform(context)
    auto_start = LaunchConfiguration("auto_start").perform(context) == "true"
    mapping_delay = float(LaunchConfiguration("mapping_delay").perform(context))
    slam_settle_time = float(LaunchConfiguration("slam_settle_time").perform(context))
    target_frame = LaunchConfiguration("target_frame").perform(context)
    start_knowledge = LaunchConfiguration("start_knowledge").perform(context) == "true"

    orchestrator = Node(
        package="arlab_movement",
        executable="movement_orchestrator",
        output="screen",
        parameters=[params_file],
    )

    # CV node for snapshot ActionServer with target_frame to map
    object_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("arlab_computer_vision"),
                "launch",
                "object_detection_launch.py",
            )
        ),
        launch_arguments={"target_frame": target_frame}.items(),
    )

    actions = [orchestrator, object_detection]

    # knowledge base launch, so CV can write entities to it
    if start_knowledge:
        knowledge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("arlab_knowledge"),
                    "launch",
                    "knowledge_launch.py",
                )
            )
        )
        actions.insert(0, knowledge)

    if auto_start:
        # start SLAM, wait for TF, then start auto annotate.
        actions.append(TimerAction(period=mapping_delay, actions=[_send_movement_goal("mapping")]))
        actions.append(
            TimerAction(
                period=mapping_delay + slam_settle_time,
                actions=[_send_movement_goal("auto_annotate")],
            )
        )

    return actions


def generate_launch_description():
    """Generate the launch description for the mapping drive bring-up."""
    default_params_file = os.path.join(
        get_package_share_directory("arlab_movement"),
        "params",
        "arlab_navigation_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Automatically send the mapping and auto_annotate goals after startup. Set false to bring up nodes only.",
            ),
            DeclareLaunchArgument(
                "start_knowledge",
                default_value="true",
                description="Start the knowledge base (database + visualization) nodes. Set false if the KB is already running elsewhere.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the movement orchestrator parameters file.",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="map",
                description="TF frame the CV node stores detected entities in; should match the SLAM map frame.",
            ),
            DeclareLaunchArgument(
                "mapping_delay",
                default_value="2.0",
                description="Seconds to wait after startup before sending the mapping (SLAM) goal.",
            ),
            DeclareLaunchArgument(
                "slam_settle_time",
                default_value="8.0",
                description="Seconds to wait after the mapping goal (for the map/odom TF to come up) before sending auto_annotate.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
