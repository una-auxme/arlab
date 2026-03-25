"""OccupancyGrid adapter class for the ARLab knowledge database.

This module contains the OccupancyGridData adapter class for converting
ROS OccupancyGrid messages to database column types and vice versa. These
adapters are used with SQLAlchemy's composite columns to store ROS
OccupancyGrid messages directly in database columns.

The adapter works by:
1. Converting ROS OccupancyGrid messages to flat column values for database storage
2. Reconstructing ROS OccupancyGrid messages when reading from the database

An OccupancyGrid message contains:
- Header information (stamp, frame_id)
- Info fields (resolution, width, height, origin)
- Data (2D array of occupancy values)

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

import array
from typing import Tuple

from nav_msgs.msg import OccupancyGrid


class OccupancyGridData:
    """Adapter to create database columns from a ROS OccupancyGrid message.

    This class enables storing a ROS OccupancyGrid message as flat database
    columns using SQLAlchemy's composite column feature. The OccupancyGrid
    message contains header information, info fields, and a 2D data array.

    The adapter works by:
    1. Storing the OccupancyGrid message in an instance variable
    2. Providing a _generate classmethod to create instances from column values
    3. Implementing __composite_values__ to extract column values from the OccupancyGrid

    Usage with SQLAlchemy:
        grid: Mapped[OccupancyGridData] = composite(
            OccupancyGridData._generate,
            mapped_column("header_stamp_nanosec", Integer),
            mapped_column("header_stamp_sec", Integer),
            ...
            mapped_column("data", LargeBinary),
        )

    Args:
        grid: The ROS OccupancyGrid message to wrap

    Attributes:
        grid: The wrapped ROS OccupancyGrid message
    """

    def __init__(self, grid: OccupancyGrid):
        """Initialize the OccupancyGridData adapter.

        Args:
            grid: The ROS OccupancyGrid message to wrap
        """
        super().__init__()
        self.grid = grid

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
        """Generate an OccupancyGridData instance from column values.

        This classmethod creates an OccupancyGridData instance and reconstructs
        a ROS OccupancyGrid message from the column values.

        Args:
            header_stamp_nanosec: Header timestamp nanoseconds
            header_stamp_sec: Header timestamp seconds
            header_frame_id: Header frame ID
            info_map_load_time_nanosec: Map load time nanoseconds
            info_map_load_time_sec: Map load time seconds
            info_resolution: Grid resolution in meters per cell
            info_width: Grid width in cells
            info_height: Grid height in cells
            info_origin_x: Origin position X
            info_origin_y: Origin position Y
            info_origin_z: Origin position Z
            info_origin_ox: Origin orientation X (quaternion)
            info_origin_oy: Origin orientation Y (quaternion)
            info_origin_oz: Origin orientation Z (quaternion)
            info_origin_ow: Origin orientation W (quaternion)
            data: Grid data as bytes

        Returns:
            An OccupancyGridData instance with a reconstructed ROS OccupancyGrid message
        """
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
        """Extract column values from the OccupancyGrid message.

        This method is called by SQLAlchemy to extract the column values
        from the OccupancyGridData instance for database storage.

        Returns:
            A tuple of all column values in the order they are defined
            in the composite column declaration
        """
        origin_pos = self.grid.info.origin.position
        origin_quat = self.grid.info.origin.orientation
        return (
            self.grid.header.stamp.nanosec,
            self.grid.header.stamp.sec,
            self.grid.header.frame_id,
            self.grid.info.map_load_time.nanosec,
            self.grid.info.map_load_time.sec,
            self.grid.info.resolution,
            self.grid.info.width,
            self.grid.info.height,
            origin_pos.x,
            origin_pos.y,
            origin_pos.z,
            origin_quat.x,
            origin_quat.y,
            origin_quat.z,
            origin_quat.w,
            self.grid.data.tobytes(),
        )
