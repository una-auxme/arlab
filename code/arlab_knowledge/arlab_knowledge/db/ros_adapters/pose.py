"""Db adapter the Pose ros message

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Tuple

from geometry_msgs.msg import Point, Pose, Quaternion
from vision_msgs.msg import Pose2D


class PoseData:
    """Adapter to create database columns from a ROS Pose"""

    def __init__(self, pose: Pose):
        super().__init__()
        self.pose = pose

    @classmethod
    def _generate(cls, x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float) -> "PoseData":
        """Generate a Pose from a row"""
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
        pos = self.pose.position
        ori = self.pose.orientation
        return pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w


class Pose2DData:
    def __init__(self, pose: Pose2D):
        super().__init__()
        self.pose = pose

    @classmethod
    def _generate(cls, x: float, y: float, theta: float) -> "Pose2DData":
        """Generate a Pose2D from a row"""
        pose = Pose2D()
        pose.position.x = x
        pose.position.y = y
        pose.theta = theta
        return Pose2DData(pose)

    def __composite_values__(
        self,
    ) -> Tuple[float, float, float]:
        return self.pose.position.x, self.pose.position.y, self.pose.theta
