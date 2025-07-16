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
    __tablename__ = "shape"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    entity_id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"), unique=True
    )
    entity: Mapped["Entity"] = relationship(back_populates="shape", single_parent=True)  # type: ignore # noqa: F821

    boundingbox2d: Mapped[Optional["BoundingBox2D"]] = relationship(
        back_populates="shape", cascade="all, delete-orphan"
    )

    pointcloud2: Mapped[Optional["PointCloud2"]] = relationship(
        back_populates="shape",
        cascade="all, delete-orphan",
    )

    @classmethod
    def from_ros_msg(cls, m: msg.Shape) -> "Shape":
        """Creates a shape from m"""
        result = cls()
        if m.has_boundingbox2d:
            result.boundingbox2d = BoundingBox2D.from_ros_msg(m.boundingbox2d)
        if m.has_pointcloud:
            result.pointcloud2 = PointCloud2.from_ros_msg(m.pointcloud)
        return result

    def to_ros_msg(self) -> msg.Shape:
        m = msg.Shape()
        if self.boundingbox2d is not None:
            m.boundingbox2d = self.boundingbox2d.to_ros_msg()
        if self.pointcloud2 is not None:
            m.pointcloud = self.pointcloud2.to_ros_msg()
        return m


class BoundingBox2D(Base):
    __tablename__ = "boundingbox2d"

    id: Mapped[int] = mapped_column(
        ForeignKey("shape.id", ondelete="CASCADE"), primary_key=True
    )
    shape: Mapped["Shape"] = relationship(
        back_populates="boundingbox2d", single_parent=True
    )

    center: Mapped[Pose2DData] = composite(
        Pose2DData._generate,
        mapped_column("x", Float),
        mapped_column("y", Float),
        mapped_column("theta", Float),
    )

    size_x: Mapped[float]
    size_y: Mapped[float]

    @classmethod
    def from_ros_msg(cls, m: BoundingBox2DMsg) -> "BoundingBox2D":
        """Creates a shape from m"""
        return cls(center=Pose2DData(m.center), size_x=m.size_x, size_y=m.size_y)

    def to_ros_msg(self) -> BoundingBox2DMsg:
        m = BoundingBox2DMsg()
        m.center = self.center.pose
        m.size_x = self.size_x
        m.size_y = self.size_y
        return m


class PointCloud2(Base):
    __tablename__ = "pointcloud2"

    id: Mapped[int] = mapped_column(
        ForeignKey("shape.id", ondelete="CASCADE"), primary_key=True
    )
    shape: Mapped["Shape"] = relationship(
        back_populates="pointcloud2", single_parent=True
    )

    height: Mapped[int]
    width: Mapped[int]

    fields: Mapped[list] = mapped_column(DBRosMsgJson([PointFieldMsg]))

    is_bigendian: Mapped[bool]
    point_step: Mapped[int]
    row_step: Mapped[int]

    data: Mapped[array.array] = mapped_column(DBInt8Data)

    is_dense: Mapped[bool]

    @classmethod
    def from_ros_msg(cls, m: PointCloud2Msg) -> "PointCloud2":
        """Creates a shape from m"""
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
        m = PointCloud2Msg()
        m.height = self.height
        m.width = self.width
        m.fields = self.fields
        m.is_bigendian = self.is_bigendian
        m.point_step = self.point_step
        m.row_step = self.row_step
        m.data = self.data
        m.is_dense = self.is_dense
        return m
