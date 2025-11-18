"""Utilities for creating sample data for testing the database.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

from arlab_knowledge_interfaces import msg
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import PointField
from sqlalchemy import inspect
from vision_msgs.msg import Pose2D

from arlab_knowledge.db.base import Base
from arlab_knowledge.db.entities.entity import Entity
from arlab_knowledge.db.entities.furniture import (
    Cupboard,
    Door,
    Furniture,
    Shelf,
    Table,
)
from arlab_knowledge.db.entities.human import Human
from arlab_knowledge.db.entities.pickable import Pickable
from arlab_knowledge.db.entities.shape import BoundingBox2D, PointCloud2, Shape
from arlab_knowledge.db.ros_adapters.pose import Pose2DData, PoseData
from arlab_knowledge.db.ros_adapters.time import TimeData
from arlab_knowledge.db.status import (
    ManipulationStatus,
    MovementStatus,
    RobotStatus,
    RobotStatusEvent,
    SafetyStatus,
)


def equality_check(b0: Base, b1: Base):
    """Checks b0 and b1 for equality

    Since b0 and b1 are database types, this function
    only takes into account the columns of the database type.

    Args:
        b0 (Base):
        b1 (Base):
    """
    assert type(b0) is type(b1), "Type equality"
    mapper = inspect(type(b0))
    for col in mapper.columns:
        name = col.name
        value0 = getattr(b0, name)
        value1 = getattr(b1, name)
        if isinstance(value0, Base) and isinstance(value1, Base):
            equality_check(value0, value1)
        else:
            assert value0 == value1, f"{name} equality: {value0}, {value1}"


def create_pose():
    return Pose(
        position=Point(x=1.0, y=2.0, z=0.5),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def create_pose2():
    return Pose(
        position=Point(x=1.01, y=5.0, z=0.7),
        orientation=Quaternion(x=0.6, y=0.02, z=0.1, w=200.0),
    )


def create_stamp():
    return Time(sec=12345, nsec=99886)


def get_entity():
    pose = create_pose()
    stamp = create_stamp()
    return Entity(
        description="TestEntity",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
    )


def get_human():
    pose = create_pose()
    stamp = create_stamp()
    return Human(
        description="TestHuman",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
    )


def get_pickable():
    pose = create_pose()
    stamp = create_stamp()
    return Pickable(
        description="TestPickable",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
        object_name=msg.EntityPickable.OBJECT_NAME_BEER,
        object_category=msg.EntityPickable.OBJECT_CATEGORY_SPHERE,
    )


def get_furniture():
    pose = create_pose()
    stamp = create_stamp()
    return Furniture(
        description="TestFurniture",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
    )


def get_cupboard():
    pose = create_pose()
    stamp = create_stamp()
    return Cupboard(
        description="TestCupboard",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
        width=5.0,
        height=1.8,
        open="true",
    )


def get_door():
    pose = create_pose()
    stamp = create_stamp()
    return Door(
        description="TestDoor",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
        width=2.0,
        open="false",
    )


def get_shelf():
    pose = create_pose()
    stamp = create_stamp()
    return Shelf(
        description="TestShelf",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
        width=7.0,
        height=1.5,
        cupboard_id=42,
    )


def get_table():
    pose = create_pose()
    stamp = create_stamp()
    return Table(
        description="TestTable",
        pose=PoseData(pose),
        pose_reference_frame="map",
        stamp=TimeData(stamp),
        height=1.5,
    )


def get_shape():
    pose2d = Pose2D()
    pose2d.position.x = 22.0
    pose2d.position.y = -1.0
    pose2d.theta = 20.0
    bounding_box = BoundingBox2D(size_x=5.0, size_y=10.0, center=Pose2DData(pose2d))
    pointcloud = PointCloud2()
    pointcloud.height = 10
    pointcloud.width = 20
    pointcloud.is_dense = True
    pointcloud.point_step = 2
    pointcloud.row_step = 3
    pointcloud.is_bigendian = True
    new_field = PointField()
    new_field.name = "chungus_power"
    new_field.datatype = 2
    new_field.offset = 2
    pointcloud.fields = [new_field]
    pointcloud.data = [1, 2, 3, 4, 5]
    shape = Shape(boundingbox2d=bounding_box, pointcloud2=pointcloud)
    return shape


def get_robot_status_event():
    stamp = create_stamp()
    status = get_manipulation_status()
    return RobotStatusEvent(stamp=TimeData(stamp), sender="TestSender", status=status)


def get_robot_status():
    return RobotStatus(is_ok=True)


def get_manipulation_status():
    return ManipulationStatus(is_ok=True)


def get_movement_status():
    return MovementStatus(is_ok=True)


def get_safety_status():
    return SafetyStatus(is_ok=True)
