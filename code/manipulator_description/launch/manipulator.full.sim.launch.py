"""
Launch file: manipulator.full.sim.launch.py
Package: manipulator_description

Full simulation stack:
    - Gazebo Sim + gz_ros2_control controllers
    - MoveIt move_group
    - MoveIt RViz

Use this when you want to plan / execute from RViz and see the real simulated
robot in Gazebo follow the executed trajectory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and compose Gazebo-control and MoveIt."""
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")
    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    launch_servo = LaunchConfiguration("launch_servo")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # Start Gazebo with ros2_control. The simple display RViz is disabled here,
    # because MoveIt starts the useful RViz instance below.
    manipulator_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_description"),
                    "launch",
                    "manipulator.control.sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "controllers_file": controllers_file,
            "description_file": description_file,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_roll": spawn_roll,
            "spawn_pitch": spawn_pitch,
            "spawn_yaw": spawn_yaw,
            "launch_rviz": "false",
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "use_mock_hardware": "true",
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
        }.items(),
    )

    # Start MoveIt move_group and the MoveIt RViz instance.
    # Important: the MoveIt launch argument is called launch_rviz_moveit,
    # not launch_rviz.
    manipulator_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("manipulator_ur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": use_sim_time,
            "launch_rviz_moveit": launch_rviz_moveit,
            "launch_servo": launch_servo,
            "publish_robot_description_semantic": publish_robot_description_semantic,
        }.items(),
    )

    return [
        manipulator_sim_control_launch,
        manipulator_moveit_launch,
    ]


def generate_launch_description():
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
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manipulator_description"), "config", "manipulator_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manipulator_description"), "urdf", "manipulator.urdf.xacro"]
            ),
            description="URDF/XACRO description file to spawn in Gazebo and publish to robot_description.",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("spawn_x", default_value="0.0", description="Initial Gazebo spawn x position."))
    declared_arguments.append(DeclareLaunchArgument("spawn_y", default_value="0.0", description="Initial Gazebo spawn y position."))
    declared_arguments.append(DeclareLaunchArgument("spawn_z", default_value="0.0", description="Initial Gazebo spawn z position."))
    declared_arguments.append(DeclareLaunchArgument("spawn_roll", default_value="0.0", description="Initial Gazebo spawn roll."))
    declared_arguments.append(DeclareLaunchArgument("spawn_pitch", default_value="0.0", description="Initial Gazebo spawn pitch."))
    declared_arguments.append(DeclareLaunchArgument("spawn_yaw", default_value="0.0", description="Initial Gazebo spawn yaw."))
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Arm controller used by MoveIt for execution in Gazebo.",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_position_controller",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the initial joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz_moveit",
            default_value="true",
            description="Start the MoveIt RViz window.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Start MoveIt Servo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time for MoveIt/RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="Let move_group publish robot_description_semantic.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
