from typing import Any, Dict, Optional

import rclpy.logging
from arlab_knowledge_interfaces import msg
from sqlalchemy import Float, Integer, String
from sqlalchemy.orm import (
    Mapped,
    composite,
    mapped_column,
    relationship,
)

import arlab_knowledge.db as db
import arlab_knowledge.db.entities as entities

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
    pose_reference_frame: Mapped[str] = mapped_column(String(100))

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Integer),
        mapped_column("stamp_sec", Integer),
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

    @classmethod
    def from_ros_msg(cls, m: msg.Entity) -> "Entity":
        """Creates an entity from m

        Note that the returned entity might be a subclass of Entity
        """
        entity_type = entities.entity_msg_type_to_class(m.entity_type)
        if entity_type is None:
            rclpy.logging.get_logger(db.DB_LOGGER_NAME).error(
                f"Received entity type '{m.entity_type}' is not supported."
                f"Base class 'Entity' will be used instead."
            )
            entity_type = Entity

        kwargs = entity_type._extract_kwargs(m)

        return entity_type(**kwargs)

    def apply_ros_msg(self, m: msg.Entity):
        """Applies values from the ros message to this entity"""
        kwargs = self._extract_kwargs(m)
        for arg, value in kwargs.items():
            setattr(self, arg, value)

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict[str, Any]:
        """Extracts all attributes for the __init__ of this type from m

        Args:
            m (msg.Entity): ROS message to extract data from

        Returns:
            Dict: arguments for the __init__ function
        """
        return {
            "stamp": TimeData(m.stamp),
            "description": m.description,
            "pose": PoseData(m.pose),
            "pose_reference_frame": m.pose_reference_frame,
        }

    def to_ros_msg(self) -> msg.Entity:
        return msg.Entity(
            entity_type=entities.entity_extract_type_msg(self),
            stamp=self.stamp.time,
            description=self.description,
            pose=self.pose.pose,
            pose_reference_frame=self.pose_reference_frame,
        )
