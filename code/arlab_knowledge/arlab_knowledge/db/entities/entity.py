"""Contains the Entity class

The Entity is the base class for all objects surrounding the robot

More documentation in the corresponding ros definitions: Entity.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Any, Dict, List, Optional

import rclpy.logging
from arlab_knowledge_interfaces import msg
from sqlalchemy import Float, Integer, String
from sqlalchemy.orm import (
    Mapped,
    composite,
    mapped_column,
    relationship,
)
from visualization_msgs.msg import Marker

import arlab_knowledge.db as db
import arlab_knowledge.db.entities as entities

from ..base import Base
from ..ros_adapters.pose import PoseData
from ..ros_adapters.time import TimeData


class Entity(Base):
    """An entity is a physical object somewhere around the robot"""

    __tablename__ = "entity"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    type: Mapped[str]
    """Entity (sub)type

    Required for database polymorphism. Do NOT set manually, use the subclasses instead.
    """

    description: Mapped[str] = mapped_column(String(100))
    """Human readable description"""

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
    """Pose of the entity relative to the pose_reference_frame"""
    pose_reference_frame: Mapped[str] = mapped_column(String(100))
    """Reference frame for the pose

    (e.g., "map", "base_link")
    """

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Integer),
        mapped_column("stamp_sec", Integer),
    )
    """Time stamp of the last update"""

    shape: Mapped[Optional["Shape"]] = relationship(  # type: ignore # noqa: F821
        back_populates="entity",
        cascade="all, delete",
        passive_deletes=True,
    )
    """Shape of this entity"""

    __mapper_args__ = {
        "polymorphic_identity": "entity",
        "polymorphic_on": "type",
    }

    def get_all_markers(self) -> List[Marker]:
        return [self.get_pose_marker()]

    def get_pose_marker(self) -> Marker:
        import arlab_common.markers

        return arlab_common.markers.debug_marker(
            base=self.pose.pose,
            frame_id=self.pose_reference_frame,
            color=(0.5, 0.5, 0.5, 0.9),
        )

    def get_meta_markers(self) -> List[Marker]:
        # TODO: Return attributes as text
        return []

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
