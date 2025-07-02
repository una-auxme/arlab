from sqlalchemy import String, Float
from sqlalchemy.orm import (
    Mapped,
    mapped_column,
    composite,
)
from nav_msgs.msg import OccupancyGrid

from .base import Base
from .ros_adapters.time import TimeData
from .ros_adapters.occupancy_grid import DBOccupancygridData


class Map(Base):
    __tablename__ = "map"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    frame_id: Mapped[str] = mapped_column(String(100))

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Float),
        mapped_column("stamp_sec", Float),
    )

    data: Mapped[OccupancyGrid] = mapped_column(DBOccupancygridData)
