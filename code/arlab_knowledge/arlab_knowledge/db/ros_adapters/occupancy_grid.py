import array
from typing import Tuple

import numpy as np
from nav_msgs.msg import OccupancyGrid


class OccupancyGridData(OccupancyGrid):
    def __init__(self, grid: OccupancyGrid):
        super().__init__()
        self.header = grid.header
        self.info = grid.info
        self.data = np.array(grid.data, dtype=np.int8)

    @classmethod
    def _generate(
        cls,
        header_stamp_nanosec: int,
        header_stamp_sec: int,
        header_frame_id: str,
        info_map_load_time_nanosec: int,
        info_map_load_time_sec: int,
        info_resolution: float,
        info_width: int,
        info_height: int,
        info_origin_x: float,
        info_origin_y: float,
        info_origin_z: float,
        info_origin_ox: float,
        info_origin_oy: float,
        info_origin_oz: float,
        info_origin_ow: float,
        data: bytes,
    ) -> "OccupancyGridData":
        """Generate a Pose from a row"""
        grid = OccupancyGrid()
        grid.header.stamp.nanosec = header_stamp_nanosec
        grid.header.stamp.sec = header_stamp_sec
        grid.header.frame_id = header_frame_id
        grid.info.map_load_time.nanosec = info_map_load_time_nanosec
        grid.info.map_load_time.sec = info_map_load_time_sec
        grid.info.resolution = info_resolution
        grid.info.width = info_width
        grid.info.height = info_height
        origin_point = grid.info.origin.position
        origin_point.x = info_origin_x
        origin_point.y = info_origin_y
        origin_point.z = info_origin_z
        origin_quaternion = grid.info.origin.orientation
        origin_quaternion.x = info_origin_ox
        origin_quaternion.y = info_origin_oy
        origin_quaternion.z = info_origin_oz
        origin_quaternion.w = info_origin_ow
        grid.data = array.array("b", data)
        return OccupancyGridData(grid)

    def __composite_values__(
        self,
    ) -> Tuple[
        int,
        int,
        str,
        int,
        int,
        float,
        int,
        int,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        bytes,
    ]:
        origin_pos = self.info.origin.position
        origin_quat = self.info.origin.orientation
        return (
            self.header.stamp.nanosec,
            self.header.stamp.sec,
            self.header.frame_id,
            self.info.map_load_time.nanosec,
            self.info.map_load_time.sec,
            self.info.resolution,
            self.info.width,
            self.info.height,
            origin_pos.x,
            origin_pos.y,
            origin_pos.z,
            origin_quat.x,
            origin_quat.y,
            origin_quat.z,
            origin_quat.w,
            self.data.tobytes(),
        )
