"""Contains the Map database schema.

More documentation in the corresponding ros definitions: GetMap.srv

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from sqlalchemy import Float, Integer, LargeBinary, String
from sqlalchemy.orm import (
    Mapped,
    composite,
    mapped_column,
)

from .base import Base
from .ros_adapters.occupancy_grid import OccupancyGridData


class Map(Base):
    """Map based on ROS OccupancyGrid message.

    Note that a ROS OccupancyGrid needs to be converted
    into OccupancyGridData when creating a Map object
    """

    __tablename__ = "map"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    grid: Mapped[OccupancyGridData] = composite(
        OccupancyGridData._generate,
        mapped_column("header_stamp_nanosec", Integer),
        mapped_column("header_stamp_sec", Integer),
        mapped_column("header_frame_id", String(100)),
        mapped_column("info_map_load_time_nanosec", Integer),
        mapped_column("info_map_load_time_sec", Integer),
        mapped_column("info_resolution", Float),
        mapped_column("info_width", Integer),
        mapped_column("info_height", Integer),
        mapped_column("info_origin_x", Float),
        mapped_column("info_origin_y", Float),
        mapped_column("info_origin_z", Float),
        mapped_column("info_origin_ox", Float),
        mapped_column("info_origin_oy", Float),
        mapped_column("info_origin_oz", Float),
        mapped_column("info_origin_ow", Float),
        mapped_column("data", LargeBinary),
    )
