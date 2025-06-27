from typing import Optional
from sqlalchemy import String, Float
from sqlalchemy.orm import (
    Mapped,
    mapped_column,
    relationship,
    composite,
)

from ..base import Base
from ..ros_adapters.pose import PoseData
from ..ros_adapters.time import TimeData


class Entity(Base):
    __tablename__ = "entity"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    type: Mapped[str]

    description: Mapped[str] = mapped_column(String(100))

    pose: Mapped[PoseData] = composite(
        PoseData._generate,
        mapped_column("x", Float),
        mapped_column("y", Float),
        mapped_column("z", Float),
        mapped_column("ox", Float),
        mapped_column("oy", Float),
        mapped_column("oz", Float),
        mapped_column("ow", Float),
    )
    frame_id: Mapped[str] = mapped_column(String(100))

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Float),
        mapped_column("stamp_sec", Float),
    )

    shape: Mapped[Optional["Shape"]] = relationship(  # type: ignore # noqa: F821
        back_populates="entity",
        cascade="all, delete",
        passive_deletes=True,
    )

    __mapper_args__ = {
        "polymorphic_identity": "entity",
        "polymorphic_on": "type",
    }
