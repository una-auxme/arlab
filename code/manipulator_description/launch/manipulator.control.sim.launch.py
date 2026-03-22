"""
Launch file: manipulator.control.sim.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Starts the Gazebo simulation with the full manipulator (UR arm + Mia Hand)
and spawns all ros2_control controllers. Does NOT start MoveIt.

Use this launch file if you only need Gazebo with controllers.
For the full simulation stack including MoveIt use manipulator.full.sim.launch.py.

Based on the original UR simulation launch file by Denis Stogl /
Stogl Robotics Consulting UG.
"""

from os.path import exists

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments, build the robot description, and create all nodes.

    Performs two runtime file-existence checks to select calibrated Mia Hand
    config files over their defaults when available. Delays all controller
    spawners until after the robot has been spawned in Gazebo.

    Args:
        context: Launch context used to resolve substitutions and perform checks.
        *args: Unused positional arguments required by OpaqueFunction.
        **kwargs: Unused keyword arguments required by OpaqueFunction.

    Returns:
        List of launch actions and nodes to execute.
    """
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    controllers_file = LaunchConfiguration("controllers_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    description_file = LaunchConfiguration("description_file")

    # Perform context-dependent arguments needed for the file-existence checks below.
    serial_port = LaunchConfiguration("serial_port").perform(context)
    laterality = LaunchConfiguration("laterality").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)

    # Use calibrated joint limits if available, otherwise fall back to defaults.
    joint_limits_config_file_path = PathJoinSubstitution(
        [FindPackageShare("mia_hand_description"), "calibration", "joint_limits.yaml"]
    ).perform(context)

    if exists(joint_limits_config_file_path):
        joint_limits_config_file = "joint_limits.yaml"
    else:
        joint_limits_config_file = "joint_limits_default.yaml"

    # Use calibrated transmission config if available, otherwise fall back to defaults.
    transmissions_config_file_path = PathJoinSubstitution(
        [FindPackageShare("mia_hand_description"), "calibration", "transmission_config.yaml"]
    ).perform(context)

    if exists(transmissions_config_file_path):
        transmissions_config_file = "transmission_config.yaml"
    else:
        transmissions_config_file = "transmission_config_default.yaml"

    # Build the robot description by running xacro on the top-level URDF file.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "simulation_controllers:=",
            controllers_file,
            " ",
            "serial_port:=",
            serial_port,
            " ",
            "laterality:=",
            laterality,
            " ",
            "prefix:=",
            prefix,
            " ",
            "joint_limits_config_file:=",
            joint_limits_config_file,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Publish the robot description and TF tree.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="both",
    )

    # Optional RViz visualisation — only started when launch_rviz:=true.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # Spawn the joint_state_broadcaster so joint states are published to /joint_states.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Publish per-finger joint states for RViz2 — only needed when RViz is active.
    rviz2_joint_state_publisher = Node(
        condition=IfCondition(launch_rviz),
        name="rviz2_joint_state_publisher",
        package="mia_hand_description",
        executable="rviz2_joint_state_publisher_node",
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare("mia_hand_description"),
                    "calibration",
                    TextSubstitution(text=transmissions_config_file),
                ]
            ),
        ],
    )

    # Spawn the primary arm controller — active or stopped depending on the argument.
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_controller),
    )

    # Spawn Mia Hand velocity controllers — started inactive so MoveIt can activate them.
    velocity_controllers_spawner = Node(
        name="velocity_controllers_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thumb_joint_velocity_controller",
            "index_joint_velocity_controller",
            "mrl_joint_velocity_controller",
            "--inactive",
            "-c",
            "/controller_manager",
        ],
    )

    # Spawn Mia Hand position controllers — started active.
    position_controllers_spawner = Node(
        name="position_controllers_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thumb_joint_position_controller",
            "index_joint_position_controller",
            "mrl_joint_position_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Spawn the Mia Hand joint trajectory controller — started inactive.
    joint_trajectory_controller_spawner = Node(
        name="joint_trajectory_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--inactive",
            "-c",
            "/controller_manager",
        ],
    )


    trajectory_controller_spawner = Node(
        name="trajectory_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--inactive",
            "-c",
            "/controller_manager",
        ],
    )

    # Spawn the robot model into the running Gazebo instance.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur5e",
            "-allow_renaming",
            "true",
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    # Start Gazebo with or without the GUI depending on the gazebo_gui argument.
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )

    # Bridge the Gazebo /clock topic into ROS so that use_sim_time works correctly.
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "--log-level",
            "error",
        ],
        output="screen",
    )

    # Delay all controller spawners until the robot has been spawned in Gazebo.
    # Without this, spawners fail because the controller manager is not yet ready.
    delay_spawners_after_gz_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                velocity_controllers_spawner,
                position_controllers_spawner,
                trajectory_controller_spawner,
                joint_trajectory_controller_spawner,
                rviz2_joint_state_publisher,
            ],
        )
    )

    nodes_to_launch = [
        robot_state_publisher_node,
        gz_launch_description,
        gz_spawn_entity,
        delay_spawners_after_gz_spawn,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        rviz_node,
        gz_sim_bridge,
    ]

    return nodes_to_launch


def generate_launch_description():
    """Declare all launch arguments and register the OpaqueFunction setup.

    Returns:
        LaunchDescription containing all declared arguments and the setup function.
    """
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur15",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([FindPackageShare("manipulator_description"), "config", "manipulator_controllers.yaml"]),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution([FindPackageShare("manipulator_description"), "urdf", "manipulator.urdf.xacro"]),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]),
            description="Rviz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"))
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file (absolute path or filename from the gazebosim worlds collection) containing a custom world.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port to which Mia Hand is connected.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "laterality",
            default_value="right",
            description="Parameter for loading a right or left hand in RViz2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix to be added before Mia Hand link and joint names.Useful for multi-robot scenarios.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
