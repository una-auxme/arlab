from typing import Optional
from sqlalchemy import Float, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, composite, relationship

from .base import Base
from .ros_adapters.time import TimeData


class MovementStatus(Base):
    __tablename__ = "robot_movement_status"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status_event.id", ondelete="CASCADE"), primary_key=True
    )
    event: Mapped["RobotStatusEvent"] = relationship(back_populates="movement")

    is_ok: Mapped[bool]


class ManipulationStatus(Base):
    __tablename__ = "robot_manipulation_status"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status_event.id", ondelete="CASCADE"), primary_key=True
    )
    event: Mapped["RobotStatusEvent"] = relationship(back_populates="manipulation")

    is_ok: Mapped[bool]


class SafetyStatus(Base):
    __tablename__ = "robot_safety_status"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status_event.id", ondelete="CASCADE"), primary_key=True
    )
    event: Mapped["RobotStatusEvent"] = relationship(back_populates="safety")

    is_ok: Mapped[bool]


class RobotStatusEvent(Base):
    __tablename__ = "robot_status_event"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Float),
        mapped_column("stamp_sec", Float),
    )

    movement: Mapped[Optional["MovementStatus"]] = relationship(
        back_populates="event", cascade="all, delete", single_parent=True
    )
    manipulation: Mapped[Optional["ManipulationStatus"]] = relationship(
        back_populates="event", cascade="all, delete", single_parent=True
    )
    safety: Mapped[Optional["SafetyStatus"]] = relationship(
        back_populates="event", cascade="all, delete", single_parent=True
    )
