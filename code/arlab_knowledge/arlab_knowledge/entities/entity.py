from typing import Optional
from sqlalchemy import ForeignKey, String
from sqlalchemy.orm import (
    Mapped,
    mapped_column,
    relationship,
    composite,
)

from arlab_knowledge import Base
from arlab_knowledge.ros_adapters.pose import PoseData
from arlab_knowledge.ros_adapters.time import TimeData

import arlab_knowledge.entities.shape as shape_module


class Entity(Base):
    __tablename__ = "entity"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    type: Mapped[str]

    description: Mapped[str] = mapped_column(String(100))

    pose: Mapped[PoseData] = composite(
        PoseData._generate, "x", "y", "z", "ox", "oy", "oz", "ow"
    )
    frame_id: Mapped[str] = mapped_column(String(100))

    stamp: Mapped[TimeData] = composite(
        TimeData._generate, "stamp_nanosec", "stamp_sec"
    )

    shape_id: Mapped[Optional[int]] = mapped_column(ForeignKey("shape.id"))
    shape: Mapped[Optional[shape_module.Shape]] = relationship(back_populates="entity")

    __mapper_args__ = {
        "polymorphic_identity": "entity",
        "polymorphic_on": "type",
    }
