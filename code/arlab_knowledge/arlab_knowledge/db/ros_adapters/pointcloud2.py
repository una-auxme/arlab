from typing import Tuple

from sensor_msgs.msg import PointCloud2, PointField


class PointCloud2Data:
    def __init__(self, pointcloud: PointCloud2):
        super().__init__()
        self.pointcloud = pointcloud

    @classmethod
    def _generate(
        cls, width: int, height: int, fields: list[PointField], data: bytes
    ) -> "PointCloud2Data":
        """Generate a PointCloud2 from a row"""
        pointcloud = PointCloud2()
        pointcloud.width = width
        pointcloud.height = height
        pointcloud.fields = fields
        pointcloud.data = data
        return PointCloud2Data(pointcloud)

    def __composite_values__(self) -> Tuple[int, int, list[PointField], bytes]:
        width = self.pointcloud.width
        height = self.pointcloud.height
        if isinstance(self.pointcloud.fields, (list, tuple, set)):
            fields = list(self.pointcloud.fields)
        else:
            fields = []
        data = self.pointcloud.data.tobytes()
        return (width, height, fields, data)
