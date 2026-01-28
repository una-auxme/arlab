from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    pitch = LaunchConfiguration("pitch")
    roll = LaunchConfiguration("roll")

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub",
        arguments=[x, y, z, yaw, pitch, roll, frame_id, child_frame_id],
    )

    return [tf_node]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="tool0",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "child_frame_id",
            default_value="camera_gripper_link",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "x",
            default_value="0",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "y",
            default_value="0",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "z",
            default_value="0",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "yaw",
            default_value="-1.57",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "roll",
            default_value="0",
            description="frame_id.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pitch",
            default_value="-1.57",
            description="frame_id.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
