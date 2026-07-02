"""Entity database schema for the ARLab knowledge database.

This module contains the Entity base class and related visualization methods
for representing physical objects around the robot. Entities can have shapes
(point clouds, bounding boxes) and are positioned in 3D space.

The Entity class uses composite columns to store ROS Pose and Time messages
as flat database columns. The shape relationship enables optional shape data
for each entity.

More documentation in the corresponding ROS definitions: Entity.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Any, Dict, List, Optional

import arlab_common.markers
import rclpy.logging
from arlab_knowledge_interfaces import msg
from geometry_msgs.msg import Vector3
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
    """An entity is a physical object somewhere around the robot.

    This is the base class for all entity types (furniture, humans, pickables, etc.).
    Each entity has:
    - A unique ID
    - A type indicating its subclass
    - A human-readable description
    - A 3D pose (position and orientation)
    - A reference frame for the pose
    - A timestamp of the last update
    - An optional shape (point cloud or bounding box)

    The ``type`` column is used for polymorphic inheritance, allowing different
    entity subclasses to be stored in the same table structure.
    """

    __tablename__ = "entity"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    """Unique identifier for this entity"""

    type: Mapped[str]
    """Entity (sub)type indicating the entity subclass.

    Required for database polymorphism. Do NOT set manually, use the subclasses instead.
    Valid values: "entity", "entity_pickable", "entity_human", "entity_furniture", etc.
    """

    description: Mapped[str] = mapped_column(String(100))
    """Human readable description of the entity.

    This field provides a text description that can be used for identification
    or display purposes (e.g., "red cup on table").
    """

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
    """3D pose of the entity.

    Uses a composite column to store the ROS Pose message as seven separate
    float columns (x, y, z, ox, oy, oz, ow). The composite pattern allows
    storing complex ROS messages as flat database columns.
    """

    pose_reference_frame: Mapped[str] = mapped_column(String(100))
    """Reference frame for the pose.

    Specifies the coordinate frame in which the pose is expressed (e.g., "map",
    "base_link", "camera_link"). This is essential for correctly interpreting
    the entity's position in the world.
    """

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Integer),
        mapped_column("stamp_sec", Integer),
    )
    """Timestamp of the last update.

    Uses a composite column to store the ROS Time message as two separate
    integer columns (nanoseconds and seconds).
    """

    shape: Mapped[Optional["Shape"]] = relationship(  # type: ignore # noqa: F821
        back_populates="entity",
        cascade="all, delete",
        passive_deletes=True,
    )
    """Shape of this entity.

    This is an optional relationship to a Shape model that contains shape
    information (point clouds, bounding boxes). The cascade configuration
    ensures that when an entity is deleted, its shape is also deleted.
    passive_deletes=True allows deleting the shape even if the relationship
    is not explicitly managed.
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity",
        "polymorphic_on": "type",
    }
    """Polymorphic mapping configuration.

    This tells SQLAlchemy to use the ``type`` column to determine which
    entity subclass to instantiate when querying the database. The
    ``polymorphic_identity`` is the value stored for this base class,
    and ``polymorphic_on`` specifies which column contains the discriminator
    value.
    """

    def get_all_markers(self, entity_id: Optional[int] = None) -> List[Marker]:
        """Returns all visualization markers for this entity.

        This method creates multiple markers for different visualization purposes:
        1. A point cloud marker (if available) showing the entity's shape
        2. A pose marker showing the entity's position
        3. A meta marker with the entity's description

        Args:
            entity_id: Optional entity ID to use as marker ID. If None, uses self.id.

        Returns:
            A list of Marker objects suitable for visualization
        """
        markers: List[Marker] = []

        point_cloud_marker = self.get_point_cloud_marker()
        if point_cloud_marker is not None:
            markers.append(point_cloud_marker)

        markers.append(self.get_pose_marker())
        markers.extend(self.get_meta_markers())
        return markers

    def get_pose_marker(self) -> Marker:
        """Creates a debug marker showing the entity's pose.

        This marker is a simple box that visualizes the entity's position
        and orientation. It uses a semi-transparent gray color.

        Returns:
            A Marker object representing the entity's pose
        """
        return arlab_common.markers.debug_marker(
            base=self.pose.pose,
            frame_id=self.pose_reference_frame,
            color=(0.5, 0.5, 0.5, 0.5),
            size_modifier=0.5,
        )

    def get_point_cloud_marker(self) -> Optional[Marker]:
        """Creates a POINTS marker from the entity's point cloud.

        This method creates a marker that visualizes the entity's shape
        as a point cloud. If no point cloud is available, returns None.

        Returns:
            A Marker with type POINTS containing the point cloud data, or None
            if no point cloud is available or conversion fails
        """
        if not self.shape or not self.shape.pointcloud2:
            return None

        # Convert PointCloud2 (DB object) to ROS msg
        pc2_msg = self.shape.pointcloud2.to_ros_msg()
        return arlab_common.markers.debug_marker(
            base=pc2_msg,
            frame_id=self.pose_reference_frame,
            color=(1.0, 1.0, 1.0, 0.9),
            size_modifier=0.005,  # 5mm
        )

    def get_meta_markers(self) -> List[Marker]:
        """Creates meta markers for the entity.

        These markers provide additional information about the entity,
        such as its description and ID. They are positioned slightly above
        the entity's pose for visibility.

        Returns:
            A list containing a single meta marker
        """
        return [
            arlab_common.markers.debug_marker(
                base=f"{self.description} ({self.id})",
                frame_id=self.pose_reference_frame,
                pose=self.pose.pose,
                offset=Vector3(x=0.0, y=0.0, z=0.05),
                color=(1.0, 1.0, 1.0, 0.9),
                size_modifier=0.5,
            )
        ]

    @classmethod
    def from_ros_msg(cls, m: msg.Entity) -> "Entity":
        """Creates an Entity instance from a ROS message.

        This method handles polymorphic instantiation - it determines the correct
        subclass based on the entity_type field in the ROS message and creates
        an instance of that class.

        Args:
            m: The ROS Entity message to convert

        Returns:
            An Entity instance (possibly a subclass) created from the message

        Note:
            If the entity_type in the message is not recognized, the base
            Entity class is used as a fallback. An error message is logged
            in this case.
        """
        entity_type = entities.entity_msg_type_to_class(m.entity_type)
        if entity_type is None:
            rclpy.logging.get_logger(db.DB_LOGGER_NAME).error(
                f"Received entity type '{m.entity_type}' is not supported. Base class 'Entity' will be used instead."
            )
            entity_type = Entity

        kwargs = entity_type._extract_kwargs(m)

        return entity_type(**kwargs)

    def apply_ros_msg(self, m: msg.Entity) -> None:
        """Applies values from a ROS message to this entity.

        This method updates the entity's attributes from a ROS message.
        It extracts the common attributes and updates them on the instance.

        Args:
            m: The ROS Entity message to apply values from
        """
        kwargs = self._extract_kwargs(m)
        for arg, value in kwargs.items():
            setattr(self, arg, value)

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict[str, Any]:
        """Extracts keyword arguments for creating an Entity instance.

        This method is part of the polymorphic pattern - each subclass overrides
        this to extract its specific attributes from the ROS message. The base
        class extracts common attributes shared by all entity types.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary of keyword arguments for the __init__ function
        """
        return {
            "stamp": TimeData(m.stamp),
            "description": m.description,
            "pose": PoseData(m.pose),
            "pose_reference_frame": m.pose_reference_frame,
        }

    def to_ros_msg(self) -> msg.Entity:
        """Converts this entity instance to a ROS message.

        This method enables serialization of the database model back to a ROS
        message. It uses the polymorphic pattern to create the appropriate
        message type based on the instance's actual class.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        return msg.Entity(
            entity_type=entities.entity_extract_type_msg(self),  # type: ignore[arg-type]
            stamp=self.stamp.time,
            description=self.description,
            pose=self.pose.pose,
            pose_reference_frame=self.pose_reference_frame,
        )
