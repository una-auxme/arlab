"""Pose adapter classes for the ARLab knowledge database.

This module contains adapter classes for converting ROS Pose messages
to database column types and vice versa. These adapters are used with
SQLAlchemy's composite columns to store ROS Pose messages directly in
database columns.

The adapters work by:
1. Converting ROS Pose messages to flat column values for database storage
2. Reconstructing ROS Pose messages when reading from the database

Adapter classes:
- PoseData: Adapter for geometry_msgs.Pose messages (3D position and orientation)
- Pose2DData: Adapter for vision_msgs.Pose2D messages (2D position and rotation)

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Tuple

from geometry_msgs.msg import Point, Pose, Quaternion
from vision_msgs.msg import Pose2D


class PoseData:
    """Adapter to create database columns from a ROS Pose message.

    This class enables storing a ROS Pose message as flat database columns
    using SQLAlchemy's composite column feature. The Pose message contains
    a 3D position (x, y, z) and orientation (quaternion ox, oy, oz, ow).

    The adapter works by:
    1. Storing the Pose message in an instance variable
    2. Providing a _generate classmethod to create instances from column values
    3. Implementing __composite_values__ to extract column values from the Pose

    Usage with SQLAlchemy:
        pose: Mapped[PoseData] = composite(
            PoseData._generate,
            mapped_column("x", Float),
            mapped_column("y", Float),
            ...
        )

    Args:
        pose: The ROS Pose message to wrap

    Attributes:
        pose: The wrapped ROS Pose message
    """

    def __init__(self, pose: Pose):
        """Initialize the PoseData adapter.

        Args:
            pose: The ROS Pose message to wrap
        """
        super().__init__()
        self.pose = pose

    @classmethod
    def _generate(
        cls,
        x: float,
        y: float,
        z: float,
        ox: float,
        oy: float,
        oz: float,
        ow: float,
    ) -> "PoseData":
        """Generate a PoseData instance from column values.

        This classmethod creates a PoseData instance and reconstructs
        a ROS Pose message from the column values.

        Args:
            x: Position x coordinate
            y: Position y coordinate
            z: Position z coordinate
            ox: Quaternion x component
            oy: Quaternion y component
            oz: Quaternion z component
            ow: Quaternion w component

        Returns:
            A PoseData instance with a reconstructed ROS Pose message
        """
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        quaternion = Quaternion()
        quaternion.x = ox
        quaternion.y = oy
        quaternion.z = oz
        quaternion.w = ow
        pose = Pose()
        pose.position = point
        pose.orientation = quaternion
        return PoseData(pose)

    def __composite_values__(
        self,
    ) -> Tuple[float, float, float, float, float, float, float]:
        """Extract column values from the Pose message.

        This method is called by SQLAlchemy to extract the column values
        from the PoseData instance for database storage.

        Returns:
            A tuple of (x, y, z, ox, oy, oz, ow) values
        """
        pos = self.pose.position
        ori = self.pose.orientation
        return pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w


class Pose2DData:
    """Adapter to create database columns from a ROS Pose2D message.

    This class enables storing a ROS Pose2D message as flat database columns
    using SQLAlchemy's composite column feature. The Pose2D message contains
    a 2D position (x, y) and rotation angle (theta).

    The adapter works by:
    1. Storing the Pose2D message in an instance variable
    2. Providing a _generate classmethod to create instances from column values
    3. Implementing __composite_values__ to extract column values from the Pose2D

    Usage with SQLAlchemy:
        pose2d: Mapped[Pose2DData] = composite(
            Pose2DData._generate,
            mapped_column("x", Float),
            mapped_column("y", Float),
            mapped_column("theta", Float),
        )

    Args:
        pose: The ROS Pose2D message to wrap

    Attributes:
        pose: The wrapped ROS Pose2D message
    """

    def __init__(self, pose: Pose2D):
        """Initialize the Pose2DData adapter.

        Args:
            pose: The ROS Pose2D message to wrap
        """
        super().__init__()
        self.pose = pose

    @classmethod
    def _generate(cls, x: float, y: float, theta: float) -> "Pose2DData":
        """Generate a Pose2DData instance from column values.

        This classmethod creates a Pose2DData instance and reconstructs
        a ROS Pose2D message from the column values.

        Args:
            x: Position x coordinate
            y: Position y coordinate
            theta: Rotation angle in radians

        Returns:
            A Pose2DData instance with a reconstructed ROS Pose2D message
        """
        pose = Pose2D()
        pose.position.x = x
        pose.position.y = y
        pose.theta = theta
        return Pose2DData(pose)

    def __composite_values__(
        self,
    ) -> Tuple[float, float, float]:
        """Extract column values from the Pose2D message.

        This method is called by SQLAlchemy to extract the column values
        from the Pose2DData instance for database storage.

        Returns:
            A tuple of (x, y, theta) values
        """
        return self.pose.position.x, self.pose.position.y, self.pose.theta
