"""Contains the Entity class

The Entity is the base class for all objects surrounding the robot

More documentation in the corresponding ros definitions: Entity.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Any, Dict, List, Optional

import rclpy.logging
from arlab_knowledge_interfaces import msg
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2 as pc2
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

    def get_all_markers(self, entity_id: int | None = None) -> List[Marker]:
        """Return markers for visualization.

        Prefer a point cloud marker if available; otherwise, fall back to
        a simple pose marker.

        Args:
            entity_id: Optional entity ID to use as marker ID. If None, uses self.id.
        """
        marker_id = entity_id if entity_id is not None else self.id
        point_cloud_marker = self.get_point_cloud_marker(marker_id=marker_id)
        if point_cloud_marker is not None:
            return [point_cloud_marker]

        # return [self.get_pose_marker(marker_id=marker_id)]
        return []

    def get_pose_marker(self, marker_id: int | None = None) -> Marker:
        import arlab_common.markers

        marker = arlab_common.markers.debug_marker(
            base=self.pose.pose,
            frame_id=self.pose_reference_frame,
            color=(0.5, 0.5, 0.5, 0.9),
        )
        # Ensure marker has unique ID and namespace
        if marker_id is not None:
            marker.id = marker_id
        elif hasattr(self, "id"):
            marker.id = self.id
        marker.ns = "knowledge_entities"
        marker.action = Marker.ADD
        return marker

    def get_point_cloud_marker(self, marker_id: int | None = None) -> Marker | None:
        """Create a POINTS marker from the entity's point cloud.

        Returns:
            Marker with type POINTS containing the point cloud data, or None
            if no point cloud is available or conversion fails.
        """
        if not self.shape or not self.shape.pointcloud2:
            return None

        try:
            # Convert PointCloud2 (DB object) to ROS msg
            pc2_msg = self.shape.pointcloud2.to_ros_msg()

            # Extract points from PointCloud2
            points: List[Point] = []
            for p in pc2.read_points(
                pc2_msg,
                field_names=["x", "y", "z"],
                skip_nans=True,
            ):
                points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))

            if not points:
                return None

            # Optional: simple downsampling if there are too many points
            max_points = 10000
            if len(points) > max_points:
                step = max(1, len(points) // max_points)
                points = points[::step]

            # Create POINTS marker
            marker = Marker()
            marker.type = Marker.POINTS
            marker.points = points
            # Use pose_reference_frame (should be "world") for marker frame_id
            # This ensures markers are displayed in the correct frame regardless of
            # the point cloud's original frame_id
            marker.header.frame_id = self.pose_reference_frame
            marker_id_val = (
                marker_id
                if marker_id is not None
                else (self.id if hasattr(self, "id") else 0)
            )
            marker.id = marker_id_val
            marker.ns = "knowledge_entities"  # Namespace for markers
            marker.action = Marker.ADD  # Explicitly set to ADD

            # Larger point size for better visibility
            marker.scale.x = 0.001  # Point size in meters (2cm)
            marker.scale.y = 0.001
            marker.scale.z = 0.001

            # Generate a unique color for this entity based on its ID
            # This ensures each entity has a distinct, consistent color
            import colorsys
            import hashlib

            # Create a hash from the entity ID to get consistent colors
            hash_obj = hashlib.md5(str(marker_id_val).encode())
            hash_int = int(hash_obj.hexdigest()[:8], 16)

            # Generate RGB values from hash using HSV color space
            # for better color distribution
            hue = (hash_int % 360) / 360.0  # 0.0 to 1.0
            saturation = 0.7 + (hash_int % 30) / 100.0  # 0.7 to 1.0
            value = 0.8 + (hash_int % 20) / 100.0  # 0.8 to 1.0

            r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)

            # Apply single color to all points
            marker.color.r = float(r)
            marker.color.g = float(g)
            marker.color.b = float(b)
            marker.color.a = 0.9  # Less transparent for better visibility

            # Debug: Log color for this entity
            rclpy.logging.get_logger("Entity").info(
                f"Point cloud marker for entity ID {marker_id_val}: "
                f"color RGB=({r:.3f}, {g:.3f}, {b:.3f}), "
                f"marker.id={marker.id}, marker.ns={marker.ns}, "
                f"frame_id='{marker.header.frame_id}'"
            )

            return marker
        except Exception as e:
            # Log error but don't crash visualization
            rclpy.logging.get_logger("Entity").warn(
                f"Failed to create point cloud marker: {e}"
            )
            return None

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
