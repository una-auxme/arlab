"""
Launch file: manipulator.control.sim.launch.py
Package: manipulator_description
Maintainer: Leonie Schmidt <leonie1.schmidt@uni-a.de>

Starts the Gazebo simulation with the full manipulator (UR arm + Mia Hand)
and spawns all ros2_control controllers. Does NOT start MoveIt.

Use this launch file if you only need Gazebo with controllers.
For the full simulation stack including MoveIt use manipulator.full.sim.launch.py.

Improved version:
- No TimerAction
- Controller spawners are started sequentially using OnProcessExit
- joint_state_broadcaster starts first
- arm trajectory controller starts after joint_state_broadcaster
- Mia Hand trajectory controller starts active for MoveIt/RViz hand execution
- Per-finger position/velocity controllers are loaded inactive to avoid interface conflicts
- duplicate joint_trajectory_controller spawner removed
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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments, build the robot description, and create all nodes."""

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

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    serial_port = LaunchConfiguration("serial_port").perform(context)
    laterality = LaunchConfiguration("laterality").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)

    # Use calibrated joint limits if available, otherwise fall back to defaults.
    joint_limits_config_file_path = PathJoinSubstitution(
        [
            FindPackageShare("mia_hand_description"),
            "calibration",
            "joint_limits.yaml",
        ]
    ).perform(context)

    if exists(joint_limits_config_file_path):
        joint_limits_config_file = "joint_limits.yaml"
    else:
        joint_limits_config_file = "joint_limits_default.yaml"

    # Use calibrated transmission config if available, otherwise fall back to defaults.
    transmissions_config_file_path = PathJoinSubstitution(
        [
            FindPackageShare("mia_hand_description"),
            "calibration",
            "transmission_config.yaml",
        ]
    ).perform(context)

    if exists(transmissions_config_file_path):
        transmissions_config_file = "transmission_config.yaml"
    else:
        transmissions_config_file = "transmission_config_default.yaml"

    # Build robot description from xacro.
    #
    # Important for the Mia hand simulation:
    # The upstream Mia URDF couples middle/ring/little fingers with mimic tags.
    # RViz/MoveIt apply those mimic tags, while Gazebo Sim executes only the
    # ros2_control joints. The wrapper below keeps ring/little mimic joints open
    # so RViz and Gazebo show the same executed hand pose.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="python3")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "scripts",
                    "filter_mia_hand_urdf.py",
                ]
            ),
            " --filter-mia-ring-little-mimic ",
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

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # More robust spawner arguments.
    # These longer timeouts help when Gazebo/world loading is slow.
    spawner_common_args = [
        "-c",
        "/controller_manager",
        "--controller-manager-timeout",
        "120",
        "--switch-timeout",
        "120",
        "--service-call-timeout",
        "120",
    ]

    # 1) Start joint_state_broadcaster first.
    joint_state_broadcaster_spawner = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ]
        + spawner_common_args,
        output="screen",
    )

    # RViz helper for Mia Hand joint states.
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
        output="screen",
    )

    # 2) Start primary UR arm controller.
    # Default: scaled_joint_trajectory_controller
    initial_joint_controller_spawner_started = Node(
        name="initial_joint_controller_spawner_started",
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
        ]
        + spawner_common_args,
        condition=IfCondition(activate_joint_controller),
        output="screen",
    )

    initial_joint_controller_spawner_inactive = Node(
        name="initial_joint_controller_spawner_inactive",
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "--inactive",
        ]
        + spawner_common_args,
        condition=UnlessCondition(activate_joint_controller),
        output="screen",
    )

    # 3) Load Mia Hand per-finger position controllers inactive.
    # RViz/MoveIt uses mia_joint_trajectory_controller below. If these single-joint
    # controllers were active, they would claim the same position interfaces and
    # block the trajectory controller.
    position_controllers_spawner = Node(
        name="position_controllers_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thumb_joint_position_controller",
            "index_joint_position_controller",
            "mrl_joint_position_controller",
            "--inactive",
        ]
        + spawner_common_args,
        output="screen",
    )

    # 4) Load Mia Hand velocity controllers inactive.
    # They use the same finger joints, so they must not be active at the same time
    # as the trajectory controller.
    velocity_controllers_spawner = Node(
        name="velocity_controllers_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "thumb_joint_velocity_controller",
            "index_joint_velocity_controller",
            "mrl_joint_velocity_controller",
            "--inactive",
        ]
        + spawner_common_args,
        output="screen",
    )

    # 5) Start Mia Hand trajectory controller active.
    # Important: this is NOT the arm joint_trajectory_controller.
    # MoveIt/RViz uses this FollowJointTrajectory action for hand_open/hand_close.
    mia_joint_trajectory_controller_spawner = Node(
        name="mia_joint_trajectory_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mia_joint_trajectory_controller",
        ]
        + spawner_common_args,
        output="screen",
    )

    # Spawn the robot model into Gazebo.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur5e",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            spawn_yaw,
            "-allow_renaming",
            "true",
            "--ros-args",
            "--log-level",
            "error",
        ],
    )

    # Start Gazebo with or without GUI.
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ros_gz_sim"),
                "/launch/gz_sim.launch.py",
            ]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[
                    " -r -v 4 ",
                    world_file,
                ],
                else_value=[
                    " -s -r -v 4 ",
                    world_file,
                ],
            )
        }.items(),
    )

    # Bridge Gazebo clock into ROS.
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

    # -------------------------------------------------------------------------
    # Controller startup order WITHOUT TimerAction:
    #
    # Gazebo spawn finished
    #   -> joint_state_broadcaster
    #      -> initial arm controller
    #         -> Mia Hand position controllers inactive
    #            -> Mia Hand velocity controllers inactive
    #               -> Mia Hand trajectory controller active
    #                  -> RViz2 Mia joint state publisher
    # -------------------------------------------------------------------------

    delay_joint_state_broadcaster_after_gz_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
            ],
        )
    )

    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                initial_joint_controller_spawner_started,
                initial_joint_controller_spawner_inactive,
            ],
        )
    )

    # If activate_joint_controller:=true, this path continues after the active arm spawner.
    delay_position_controllers_after_active_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_spawner_started,
            on_exit=[
                position_controllers_spawner,
            ],
        )
    )

    # If activate_joint_controller:=false, this path continues after the inactive arm spawner.
    delay_position_controllers_after_inactive_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_spawner_inactive,
            on_exit=[
                position_controllers_spawner,
            ],
        )
    )

    delay_velocity_controllers_after_position_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controllers_spawner,
            on_exit=[
                velocity_controllers_spawner,
            ],
        )
    )

    delay_mia_trajectory_controller_after_velocity_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_controllers_spawner,
            on_exit=[
                mia_joint_trajectory_controller_spawner,
            ],
        )
    )

    delay_rviz2_joint_state_publisher_after_mia_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mia_joint_trajectory_controller_spawner,
            on_exit=[
                rviz2_joint_state_publisher,
            ],
        )
    )

    nodes_to_launch = [
        robot_state_publisher_node,
        gz_launch_description,
        gz_sim_bridge,
        gz_spawn_entity,
        delay_joint_state_broadcaster_after_gz_spawn,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_position_controllers_after_active_arm_controller,
        delay_position_controllers_after_inactive_arm_controller,
        delay_velocity_controllers_after_position_controllers,
        delay_mia_trajectory_controller_after_velocity_controllers,
        delay_rviz2_joint_state_publisher_after_mia_trajectory_controller,
        rviz_node,
    ]

    return nodes_to_launch


def generate_launch_description():
    """Declare all launch arguments and register the OpaqueFunction setup."""

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
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "config",
                    "manipulator_controllers.yaml",
                ]
            ),
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
            description="Start the initial joint controller as active.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Robot arm controller to start.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "urdf",
                    "manipulator.urdf.xacro",
                ]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Initial Gazebo spawn x position.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Initial Gazebo spawn y position.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Initial Gazebo spawn z position.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_roll",
            default_value="0.0",
            description="Initial Gazebo spawn roll.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_pitch",
            default_value="0.0",
            description="Initial Gazebo spawn pitch.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Initial Gazebo spawn yaw.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_description"),
                    "rviz",
                    "view_robot.rviz",
                ]
            ),
            description="RViz config file to use.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description=("Gazebo world file. Can be an absolute path or a filename from the Gazebo worlds collection."),
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
            description="Parameter for loading a right or left hand.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix to be added before Mia Hand link and joint names.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
