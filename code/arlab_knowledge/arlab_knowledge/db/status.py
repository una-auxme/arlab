from typing import Any, Dict

import rclpy.logging
from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey, Integer, String
from sqlalchemy.orm import Mapped, composite, mapped_column, relationship

import arlab_knowledge.db as db

from .base import Base
from .ros_adapters.time import TimeData


def status_msg_type_to_class(msg_type: msg.StatusType):
    if msg_type.id == msg.StatusType.STATUS:
        return RobotStatus
    if msg_type.id == msg.StatusType.STATUS_MANIPULATION:
        return ManipulationStatus
    if msg_type.id == msg.StatusType.STATUS_MOVEMENT:
        return MovementStatus
    if msg_type.id == msg.StatusType.STATUS_SAFETY:
        return SafetyStatus
    else:
        return None


def status_extract_type_msg(e: "RobotStatus") -> msg.StatusType:
    m = msg.StatusType()
    m.id = msg.StatusType.STATUS
    if isinstance(e, ManipulationStatus):
        m.id = msg.StatusType.STATUS_MANIPULATION
    if isinstance(e, MovementStatus):
        m.id = msg.StatusType.STATUS_MOVEMENT
    if isinstance(e, SafetyStatus):
        m.id = msg.StatusType.STATUS_SAFETY
    return m


class RobotStatus(Base):
    __tablename__ = "robot_status"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    type: Mapped[str]

    event: Mapped["RobotStatusEvent"] = relationship(
        back_populates="status", cascade="all, delete"
    )

    is_ok: Mapped[bool]

    __mapper_args__ = {
        "polymorphic_identity": "robot_status",
        "polymorphic_on": "type",
    }

    @classmethod
    def from_ros_msg(cls, m: msg.RobotStatus) -> "RobotStatus":
        """Creates a RobotStatus from m

        Note that the returned status might be a subclass of RobotStatus
        """
        status_type = status_msg_type_to_class(m.status_type)
        if status_type is None:
            rclpy.logging.get_logger(db.DB_LOGGER_NAME).error(
                f"Received entity type '{m.status_type}' is not supported."
                f"Base class 'Entity' will be used instead."
            )
            status_type = RobotStatus

        kwargs = status_type._extract_kwargs(m)

        return status_type(**kwargs)

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict[str, Any]:
        """Extracts all attributes for the __init__ of this type from m

        Args:
            m (msg.RobotStatus): ROS message to extract data from

        Returns:
            Dict: arguments for the __init__ function
        """
        return {"is_ok": m.is_ok}

    def to_ros_msg(self) -> msg.RobotStatus:
        return msg.RobotStatus(
            status_type=status_extract_type_msg(self), is_ok=self.is_ok
        )


class MovementStatus(RobotStatus):
    __tablename__ = "robot_status_movement"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"), primary_key=True
    )

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_movement",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Assign subclass specific attributes here
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        return m


class ManipulationStatus(RobotStatus):
    __tablename__ = "robot_status_manipulation"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"), primary_key=True
    )

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_manipulation",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Assign subclass specific attributes here
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        return m


class SafetyStatus(RobotStatus):
    __tablename__ = "robot_status_safety"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"), primary_key=True
    )

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_safety",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Assign subclass specific attributes here
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        return m


class RobotStatusEvent(Base):
    __tablename__ = "robot_status_event"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Integer),
        mapped_column("stamp_sec", Integer),
    )

    sender: Mapped[str] = mapped_column(String(100))

    status_id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE")
    )
    status: Mapped["RobotStatus"] = relationship(
        back_populates="event", cascade="all, delete", single_parent=True
    )

    @classmethod
    def from_ros_msg(cls, m: msg.RobotStatusEvent) -> "RobotStatusEvent":
        """Creates a status event from m"""
        status = RobotStatus.from_ros_msg(m.status)
        return cls(stamp=TimeData(m.stamp), sender=m.sender, status=status)

    def to_ros_msg(self) -> msg.RobotStatusEvent:
        status = self.status.to_ros_msg()
        return msg.RobotStatusEvent(
            stamp=self.stamp.time, sender=self.sender, status=status
        )
