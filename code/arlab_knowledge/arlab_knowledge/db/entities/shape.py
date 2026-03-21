"""Shape database schema for the ARLab knowledge database.

This module contains the Shape class and related classes for representing
the geometric shape of entities. Shapes include:
- Shape: Base class for entity shapes
- BoundingBox2D: 2D bounding box for camera images
- PointCloud2: 3D point cloud data

Shapes are used for:
- Analyzing entities (e.g., finding gripping points)
- Visualizing entities in the database

More documentation in the corresponding ROS definitions: Shape.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

import array
from typing import Optional

from arlab_knowledge_interfaces import msg
from sensor_msgs.msg import PointCloud2 as PointCloud2Msg
from sensor_msgs.msg import PointField as PointFieldMsg
from sqlalchemy import Float, ForeignKey
from sqlalchemy.orm import Mapped, composite, mapped_column, relationship
from vision_msgs.msg import BoundingBox2D as BoundingBox2DMsg

from ..base import Base
from ..ros_adapters.int8_data import DBInt8Data
from ..ros_adapters.json_conv import DBRosMsgJson
from ..ros_adapters.pose import Pose2DData


class Shape(Base):
    """Shape of an entity.

    This class represents the geometric shape information for an entity.
    Each entity can have one of the following shape types:
    - A 2D bounding box (from camera images)
    - A 3D point cloud (from depth sensors)
    - Both (if available)

    The shape is used for:
    - Analyzing the entity (e.g., finding gripping points)
    - Visualizing the entity in the database

    The relationship with Entity is configured with single_parent=True to
    ensure that each shape belongs to exactly one entity.
    """

    __tablename__ = "shape"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    """Unique identifier for this shape record"""

    entity_id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"),
        unique=True,
    )
    """Foreign key to the parent entity.

    This links the shape to its parent entity. The unique constraint ensures
    that each entity has at most one shape record. When the entity is deleted,
    this shape is also deleted (CASCADE).
    """

    entity: Mapped["Entity"] = relationship(  # type: ignore # noqa: F821
        back_populates="shape",
        single_parent=True,
    )
    """Entity this shape belongs to.

    This is a bidirectional relationship with the Entity model.
    """

    boundingbox2d: Mapped[Optional["BoundingBox2D"]] = relationship(
        back_populates="shape",
        cascade="all, delete-orphan",
    )
    """2D Camera bounding box.

    This represents a bounding box detected from a 2D camera image.
    """

    pointcloud2: Mapped[Optional["PointCloud2"]] = relationship(
        back_populates="shape",
        cascade="all, delete-orphan",
    )
    """3D point cloud.

    This represents a 3D point cloud detected from a depth sensor.
    """

    @classmethod
    def from_ros_msg(cls, m: msg.Shape) -> "Shape":
        """Creates a Shape instance from a ROS message.

        This method handles optional shape data - it creates a Shape instance
        and populates it with bounding box and/or point cloud data if available
        in the ROS message.

        Args:
            m: The ROS Shape message to convert

        Returns:
            A Shape instance created from the message
        """
        result = cls()
        if m.has_boundingbox2d:
            result.boundingbox2d = BoundingBox2D.from_ros_msg(m.boundingbox2d)
        if m.has_pointcloud:
            result.pointcloud2 = PointCloud2.from_ros_msg(m.pointcloud)
        return result

    def to_ros_msg(self) -> msg.Shape:
        """Converts this shape instance to a ROS message.

        This method serializes the shape back to a ROS message, including
        only the shape data that is present (bounding box, point cloud, or neither).

        Returns:
            A ROS Shape message populated with this instance's data
        """
        m = msg.Shape()
        if self.boundingbox2d is not None:
            m.boundingbox2d = self.boundingbox2d.to_ros_msg()
            m.has_boundingbox2d = True
        if self.pointcloud2 is not None:
            m.pointcloud = self.pointcloud2.to_ros_msg()
            m.has_pointcloud = True
        return m


class BoundingBox2D(Base):
    """2D bounding box as a database table/schema.

    This class represents a 2D bounding box detected from a camera image.
    It stores the center position (x, y, theta) and size (width, height)
    of the bounding box in the camera's coordinate frame.

    The relationship with Shape is configured with single_parent=True to
    ensure that each bounding box belongs to exactly one shape.
    """

    __tablename__ = "boundingbox2d"

    id: Mapped[int] = mapped_column(
        ForeignKey("shape.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent shape record.

    This column serves as the primary key for the boundingbox2d table while
    also linking to the parent Shape record. When the parent shape is deleted,
    this bounding box record is also deleted (CASCADE).
    """

    shape: Mapped["Shape"] = relationship(
        back_populates="boundingbox2d",
        single_parent=True,
    )
    """Shape this bounding box belongs to.

    This is a bidirectional relationship with the Shape model.
    """

    center: Mapped[Pose2DData] = composite(
        Pose2DData._generate,
        mapped_column("x", Float),
        mapped_column("y", Float),
        mapped_column("theta", Float),
    )
    """Center of the bounding box.

    Uses a composite column to store the Pose2D message as three separate
    float columns (x, y, theta). The center is expressed in the camera's
    coordinate frame.
    """

    size_x: Mapped[float]
    """Width of the bounding box in meters."""

    size_y: Mapped[float]
    """Height of the bounding box in meters."""

    @classmethod
    def from_ros_msg(cls, m: BoundingBox2DMsg) -> "BoundingBox2D":
        """Creates a BoundingBox2D instance from a ROS message.

        Args:
            m: The ROS BoundingBox2D message to convert

        Returns:
            A BoundingBox2D instance created from the message
        """
        return cls(
            center=Pose2DData(m.center),
            size_x=m.size_x,
            size_y=m.size_y,
        )

    def to_ros_msg(self) -> BoundingBox2DMsg:
        """Converts this bounding box instance to a ROS message.

        Returns:
            A ROS BoundingBox2D message populated with this instance's data
        """
        m = BoundingBox2DMsg()
        m.center = self.center.pose
        m.size_x = self.size_x
        m.size_y = self.size_y
        return m


class PointCloud2(Base):
    """3D point cloud as a database table/schema.

    This class represents a 3D point cloud detected from a depth sensor.
    It stores the point cloud metadata (dimensions, field information) and
    the actual point data as a byte array.

    The point cloud data is stored using a TypeDecorator that converts
    the byte array to/from a signed integer array for database storage.
    When reading from the database, the data is converted back to an
    unsigned byte array as expected by ROS.

    The relationship with Shape is configured with single_parent=True to
    ensure that each point cloud belongs to exactly one shape.
    """

    __tablename__ = "pointcloud2"

    id: Mapped[int] = mapped_column(
        ForeignKey("shape.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent shape record.

    This column serves as the primary key for the pointcloud2 table while
    also linking to the parent Shape record. When the parent shape is deleted,
    this point cloud record is also deleted (CASCADE).
    """

    shape: Mapped["Shape"] = relationship(
        back_populates="pointcloud2",
        single_parent=True,
    )
    """Shape this point cloud belongs to.

    This is a bidirectional relationship with the Shape model.
    """

    height: Mapped[int]
    """Height of the point cloud in points."""

    width: Mapped[int]
    """Width of the point cloud in points."""

    fields: Mapped[list] = mapped_column(DBRosMsgJson([PointFieldMsg]))
    """Point field information as JSON.

    Stores the point cloud field definitions (e.g., position, intensity,
    ring) as a JSON array. Each field has a name, offset, datatype, and
    datatype size.
    """

    is_bigendian: Mapped[bool]
    """Whether the point cloud data is stored in big-endian format."""

    point_step: Mapped[int]
    """Byte offset of the start of the next point.

    This is the total size in bytes of one point, including all fields.
    """

    row_step: Mapped[int]
    """Byte offset of the start of the next row.

    This is the total size in bytes of one row, including all points.
    """

    data: Mapped[array.array] = mapped_column(DBInt8Data)
    """Point cloud data as a byte array.

    This stores the actual point cloud data. Each point is represented
    by a sequence of bytes corresponding to the fields defined in the
    fields array.
    """

    is_dense: Mapped[bool]
    """Whether the point cloud is dense (no missing points).

    Dense point clouds have a regular grid structure, while sparse
    point clouds may have missing points.
    """

    @classmethod
    def from_ros_msg(cls, m: PointCloud2Msg) -> "PointCloud2":
        """Creates a PointCloud2 instance from a ROS message.

        Args:
            m: The ROS PointCloud2 message to convert

        Returns:
            A PointCloud2 instance created from the message
        """
        return cls(
            height=m.height,
            width=m.width,
            fields=m.fields,
            is_bigendian=m.is_bigendian,
            point_step=m.point_step,
            row_step=m.row_step,
            data=m.data,
            is_dense=m.is_dense,
        )

    def to_ros_msg(self) -> PointCloud2Msg:
        """Converts this point cloud instance to a ROS message.

        This method handles the byte array conversion - DBInt8Data stores
        data as signed bytes ('b'), but ROS expects unsigned bytes ('B').
        The data is converted accordingly.

        Returns:
            A ROS PointCloud2 message populated with this instance's data
        """
        m = PointCloud2Msg()
        m.height = self.height
        m.width = self.width
        m.fields = self.fields
        m.is_bigendian = self.is_bigendian
        m.point_step = self.point_step
        m.row_step = self.row_step
        # Convert signed byte array to unsigned byte array
        # DBInt8Data stores data as signed bytes ('b'),
        # but ROS expects unsigned bytes ('B')
        if isinstance(self.data, array.array) and self.data.typecode == "b":
            # Convert signed bytes (-128 to 127) to unsigned bytes (0 to 255)
            m.data = array.array("B", (b & 0xFF for b in self.data))
        else:
            m.data = self.data
        m.is_dense = self.is_dense
        return m
