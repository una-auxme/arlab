from typing import Tuple

from geometry_msgs.msg import Point, Pose, Quaternion


class PoseData:
    def __init__(self, pose: Pose):
        super().__init__()
        self.pose = pose

    @classmethod
    def _generate(
        cls, x: float, y: float, z: float, ox: float, oy: float, oz: float, ow: float
    ) -> "PoseData":
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
