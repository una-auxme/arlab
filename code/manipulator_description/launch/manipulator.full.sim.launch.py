"""
Launch file: manipulator.full.sim.launch.py
Package: manipulator_description

It starts in one go:
- Gazebo Sim with the mobile URDF (manipulator_mobile.urdf.xacro)
- gz_ros2_control controllers for the UR arm
- MoveIt move_group + MoveIt RViz for arm planning/execution
- Gazebo ↔ ROS 2 bridge for the mobile base and lidar
- SLAM Toolbox (async)
- Nav2 navigation stack
- Optional: Nav2 RViz view

Use this when you want a single entry point for the full mobile manipulator
simulation with mapping and navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Resolve launch arguments and compose the full simulation stack."""
    # Package shares (paths to launch/config files)
    manipulator_pkg_share = FindPackageShare("manipulator_description")
    house_pkg_share = FindPackageShare("small_house")
    movement_pkg_share = FindPackageShare("arlab_movement")

    # Path to small_house models (for Gazebo resource search)
    house_models_path = PathJoinSubstitution([house_pkg_share, "models"])
    # URDF for the mobile manipulator (Ranger base + UR arm)
    mobile_description = PathJoinSubstitution([manipulator_pkg_share, "urdf", "manipulator_mobile.urdf.xacro"])

    # -------------------------------------------------------------------------
    # Robot and controller configuration (passed to Gazebo/MoveIt)
    # -------------------------------------------------------------------------
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    controllers_file = LaunchConfiguration("controllers_file")
    world_file = LaunchConfiguration("world_file")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # -------------------------------------------------------------------------
    # Spawn pose in Gazebo (x, y, z, roll, pitch, yaw)
    # -------------------------------------------------------------------------
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # -------------------------------------------------------------------------
    # SLAM / Nav2 toggles (optional components)
    # -------------------------------------------------------------------------
    launch_nav2 = LaunchConfiguration("launch_nav2")
    launch_slam = LaunchConfiguration("launch_slam")
    launch_nav2_rviz = LaunchConfiguration("launch_nav2_rviz")

    # -------------------------------------------------------------------------
    # Gazebo + ros2_control (simulated hardware + controllers)
    # -------------------------------------------------------------------------
    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [manipulator_pkg_share, "launch", "manipulator.control.sim.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "controllers_file": controllers_file,
            "description_file": mobile_description,  # mobile URDF with Ranger base
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_roll": spawn_roll,
            "spawn_pitch": spawn_pitch,
            "spawn_yaw": spawn_yaw,
            "launch_rviz": "false",  # no simple RViz here; MoveIt starts its own
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "use_mock_hardware": "true",  # use simulated hardware via gz_ros2_control
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
        }.items(),
    )

    # -------------------------------------------------------------------------
    # MoveIt stack (move_group + RViz for planning/execution)
    # -------------------------------------------------------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("manipulator_ur_moveit_config"), "launch", "ur_moveit.launch.py"]
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

    # -------------------------------------------------------------------------
    # SLAM Toolbox (async mapping: map -> odom)
    # -------------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([movement_pkg_share, "launch", "slam_async.launch.py"])
        ),
        condition=IfCondition(launch_slam),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": PathJoinSubstitution(
                [movement_pkg_share, "params", "ranger_slam_params.yaml"]
            ),
        }.items(),
    )

    # -------------------------------------------------------------------------
    # Nav2 navigation stack (path planning + controller, uses /scan and /odom)
    # -------------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([movement_pkg_share, "launch", "ranger_nav2.launch.py"])
        ),
        condition=IfCondition(launch_nav2),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": PathJoinSubstitution(
                [movement_pkg_share, "params", "ranger_nav2_params.yaml"]
            ),
        }.items(),
    )

    # -------------------------------------------------------------------------
    # RViz configured for Nav2 default view (map, paths, robot pose)
    # -------------------------------------------------------------------------
    nav2_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        output="log",
        arguments=["-d", PathJoinSubstitution([movement_pkg_share, "rviz", "nav2_default_view.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_nav2_rviz),
    )

    # -------------------------------------------------------------------------
    # Gazebo transport ↔ ROS 2 bridges for the mobile base and lidar
    # Bridges:
    #   /cmd_vel  (ROS → Gazebo)  for base velocity commands
    #   /odom     (Gazebo → ROS)  for odometry
    #   /tf       (Gazebo → ROS)  for transforms
    #   /scan     (Gazebo → ROS)  for 2D lidar scans
    # -------------------------------------------------------------------------
    mobile_bridge = Node(
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
    )

    # -------------------------------------------------------------------------
    # Static TF: scoped lidar frame for the mobile base
    # Transforms: front_lidar_link -> ur5e/ranger_base_link/front_lidar
    # -------------------------------------------------------------------------
    static_tf = Node(
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
    )

    # Assemble the full list of actions to execute
    return [
        # Gazebo resource paths (so small_house models are found)
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[house_models_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[house_models_path, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        sim_control_launch,
        moveit_launch,
        slam_launch,
        nav2_launch,
        nav2_rviz_node,
        mobile_bridge,
        static_tf,
    ]


def generate_launch_description():
    declared_arguments = []

    manipulator_pkg_share = FindPackageShare("manipulator_description")
    house_pkg_share = FindPackageShare("small_house")

    default_house_world = PathJoinSubstitution([house_pkg_share, "worlds", "small_house.world"])

    # UR robot type
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=[
                "ur3", "ur3e", "ur5", "ur5e", "ur7e", "ur10", "ur10e", "ur12e", "ur16e", "ur15", "ur20", "ur30",
            ],
        )
    )
    # Safety controller toggle
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # Controllers YAML (joint controllers, etc.)
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [manipulator_pkg_share, "config", "manipulator_controllers.yaml"]
            ),
            description="Absolute path to YAML file with the controllers configuration.",
        )
    )
    # Gazebo world file
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value=default_house_world,
            description="World file to load. Defaults to the bundled small_house world.",
        )
    )
    # Gazebo GUI on/off
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start Gazebo with GUI.",
        )
    )
    # MoveIt RViz on/off
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz_moveit",
            default_value="true",
            description="Start MoveIt RViz for planning and execution.",
        )
    )
    # MoveIt Servo (teleop) on/off
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo",
            default_value="false",
            description="Start MoveIt Servo.",
        )
    )
    # Simulation time flag (for clocks)
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation time for MoveIt/RViz.",
        )
    )
    # Initial joint controller for the arm
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
    # Activate the initial controller immediately
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate the initial joint controller.",
        )
    )
    # Publish semantic description for MoveIt
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_description_semantic",
            default_value="true",
            description="Let move_group publish robot_description_semantic.",
        )
    )

    # Spawn position/orientation for the mobile base
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="-4.0",
            description="Initial mobile base x position.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="1.0",
            description="Initial mobile base y position.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Initial mobile base z position.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_roll",
            default_value="0.0",
            description="Initial mobile base roll.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_pitch",
            default_value="0.0",
            description="Initial mobile base pitch.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Initial mobile base yaw.",
        )
    )

    # Optional: Nav2 stack
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_nav2",
            default_value="true",
            description="Start Nav2 navigation stack.",
        )
    )
    # Optional: SLAM Toolbox
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_slam",
            default_value="true",
            description="Start SLAM Toolbox.",
        )
    )
    # Optional: RViz with Nav2 view
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_nav2_rviz",
            default_value="true",
            description="Start RViz with Nav2 default view.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])