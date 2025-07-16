from arlab_knowledge_interfaces import msg
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, Quaternion
from sqlalchemy import inspect

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
from arlab_knowledge.db.ros_adapters.pose import PoseData
from arlab_knowledge.db.ros_adapters.time import TimeData
from arlab_knowledge.db.status import (
    ManipulationStatus,
    MovementStatus,
    RobotStatus,
    RobotStatusEvent,
    SafetyStatus,
)


def equality_check(b0: Base, b1: Base):
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
        picking_tag=msg.EntityPickable.TAG_PRINGLES,
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
