"""Robot status database schemata for the ARLab knowledge database.

This module contains SQLAlchemy models for robot status information, including:
- RobotStatus: Base class for all status messages
- MovementStatus: Status from the movement subsystem
- ManipulationStatus: Status from the manipulation subsystem
- SafetyStatus: Status from the safety subsystem
- RobotStatusEvent: Events containing status messages

The status system uses polymorphic inheritance to handle different status types
while maintaining a unified database schema. Each status type corresponds to a
ROS message (RobotStatus.msg) with a status_type field indicating the subtype.

More documentation in the corresponding ROS definitions:
- RobotStatus.msg: Main status message
- Status[].msg: RobotStatus subclasses
- RobotStatusEvent.msg: Status event message

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Any, Dict

import rclpy.logging
from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey, Integer, String
from sqlalchemy.orm import Mapped, composite, mapped_column, relationship

import arlab_knowledge.db as db

from .base import Base
from .ros_adapters.time import TimeData


def status_msg_type_to_class(msg_type: msg.StatusType) -> type | None:
    """Converts a StatusType message to a corresponding status class.

    This function enables polymorphic instantiation of status subclasses
    based on the ROS message status_type field. When a status event arrives,
    this function determines which Python class to instantiate.

    Args:
        msg_type: The StatusType message to convert

    Returns:
        The corresponding status class, or None if the type is not supported
    """
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
    """Extracts a StatusType message from a RobotStatus instance.

    This function enables polymorphic serialization of status instances back
    to ROS messages. It determines the appropriate StatusType based on the
    status instance's actual class type.

    Args:
        e: The RobotStatus instance to extract the type from

    Returns:
        A populated StatusType message with the appropriate id
    """
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
    """Generic robot status message.

    This is the base class for all robot status messages. Subclasses represent
    different subsystem statuses (movement, manipulation, safety). The database
    uses polymorphic inheritance to store different status types in separate
    tables while maintaining a unified schema.

    The ``type`` column stores the polymorphic identity string, which determines
    which subclass is being used. This enables the database to distinguish between
    different status types at the row level.
    """

    __tablename__ = "robot_status"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    """Unique identifier for this status record"""

    type: Mapped[str]
    """Entity (sub)type indicating the status subclass.

    Required for database polymorphism. Do NOT set manually, use the subclasses instead.
    Valid values: "robot_status", "robot_status_movement", "robot_status_manipulation", "robot_status_safety"
    """

    event: Mapped["RobotStatusEvent"] = relationship(
        back_populates="status",
        cascade="all, delete",
    )
    """Associated status event.

    When this status is deleted, its associated event is also deleted.
    """

    is_ok: Mapped[bool]
    """If the robot is in an ok state (no fatal error).

    This attribute is subject to change and indicates whether the robot
    is operating normally or has encountered an error condition.
    """

    __mapper_args__ = {
        "polymorphic_identity": "robot_status",
        "polymorphic_on": "type",
    }
    """Polymorphic mapping configuration.

    This tells SQLAlchemy to use the ``type`` column to determine which
    subclass to instantiate when querying the database. The ``polymorphic_identity``
    is the value stored for this base class, and ``polymorphic_on`` specifies
    which column contains the discriminator value.
    """

    @classmethod
    def from_ros_msg(cls, m: msg.RobotStatus) -> "RobotStatus":
        """Creates a RobotStatus instance from a ROS message.

        This method handles polymorphic instantiation - it determines the correct
        subclass based on the status_type field in the ROS message and creates
        an instance of that class.

        Args:
            m: The ROS RobotStatus message to convert

        Returns:
            A RobotStatus instance (possibly a subclass) created from the message

        Note:
            If the status_type in the message is not recognized, the base
            RobotStatus class is used as a fallback. An error message is logged
            in this case.
        """
        status_type = status_msg_type_to_class(m.status_type)
        if status_type is None:
            rclpy.logging.get_logger(db.DB_LOGGER_NAME).error(
                f"Received status type '{m.status_type}' is not supported. Base class 'RobotStatus' will be used instead."
            )
            status_type = RobotStatus

        kwargs = status_type._extract_kwargs(m)

        return status_type(**kwargs)

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict[str, Any]:
        """Extracts keyword arguments for creating a RobotStatus instance.

        This method is part of the polymorphic pattern - each subclass overrides
        this to extract its specific attributes from the ROS message. The base
        class extracts common attributes shared by all status types.

        Args:
            m: The ROS RobotStatus message to extract data from

        Returns:
            A dictionary of keyword arguments for the __init__ function
        """
        return {"is_ok": m.is_ok}

    def to_ros_msg(self) -> msg.RobotStatus:
        """Converts this status instance to a ROS message.

        This method enables serialization of the database model back to a ROS
        message. It uses the polymorphic pattern to create the appropriate
        message type based on the instance's actual class.

        Returns:
            A ROS RobotStatus message populated with this instance's data
        """
        return msg.RobotStatus(status_type=status_extract_type_msg(self), is_ok=self.is_ok)


class MovementStatus(RobotStatus):
    """Status message from the movement subsystem.

    This subclass represents status information from the robot's movement
    subsystem (e.g., locomotion, navigation). It inherits all attributes from
    RobotStatus and adds movement-specific fields if needed.
    """

    __tablename__ = "robot_status_movement"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent status record.

    This column serves as the primary key for the movement status table
    while also linking to the parent RobotStatus record. When the parent
    is deleted, this record is also deleted (CASCADE).
    """

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_movement",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        """Extracts keyword arguments for creating a MovementStatus instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then would add movement-specific attributes if any existed. Currently
        movement status shares the same attributes as the base class.

        Args:
            m: The ROS RobotStatus message to extract data from

        Returns:
            A dictionary containing at least ``is_ok`` from the parent class
        """
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Movement status uses the same attributes as the base class
        # Add movement-specific attributes here if needed in the future
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        """Converts this movement status to a ROS message.

        This method calls the parent's to_ros_msg and would add movement-specific
        data if any existed. Currently movement status shares the same message
        structure as the base class.

        Returns:
            A ROS RobotStatus message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Movement status uses the same message structure as the base class
        # Add movement-specific fields here if needed in the future
        return m


class ManipulationStatus(RobotStatus):
    """Status message from the manipulation subsystem.

    This subclass represents status information from the robot's manipulation
    subsystem (e.g., gripper operations, object handling). It inherits all
    attributes from RobotStatus and adds manipulation-specific fields if needed.
    """

    __tablename__ = "robot_status_manipulation"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent status record.

    This column serves as the primary key for the manipulation status table
    while also linking to the parent RobotStatus record. When the parent
    is deleted, this record is also deleted (CASCADE).
    """

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_manipulation",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        """Extracts keyword arguments for creating a ManipulationStatus instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then would add manipulation-specific attributes if any existed. Currently
        manipulation status shares the same attributes as the base class.

        Args:
            m: The ROS RobotStatus message to extract data from

        Returns:
            A dictionary containing at least ``is_ok`` from the parent class
        """
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Manipulation status uses the same attributes as the base class
        # Add manipulation-specific attributes here if needed in the future
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        """Converts this manipulation status to a ROS message.

        This method calls the parent's to_ros_msg and would add manipulation-specific
        data if any existed. Currently manipulation status shares the same message
        structure as the base class.

        Returns:
            A ROS RobotStatus message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Manipulation status uses the same message structure as the base class
        # Add manipulation-specific fields here if needed in the future
        return m


class SafetyStatus(RobotStatus):
    """Status message from the safety subsystem.

    This subclass represents status information from the robot's safety
    subsystem (e.g., emergency stops, safety violations). It inherits all
    attributes from RobotStatus and adds safety-specific fields if needed.
    """

    __tablename__ = "robot_status_safety"
    id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent status record.

    This column serves as the primary key for the safety status table
    while also linking to the parent RobotStatus record. When the parent
    is deleted, this record is also deleted (CASCADE).
    """

    __mapper_args__ = {
        "polymorphic_identity": "robot_status_safety",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.RobotStatus) -> Dict:
        """Extracts keyword arguments for creating a SafetyStatus instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then would add safety-specific attributes if any existed. Currently
        safety status shares the same attributes as the base class.

        Args:
            m: The ROS RobotStatus message to extract data from

        Returns:
            A dictionary containing at least ``is_ok`` from the parent class
        """
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Safety status uses the same attributes as the base class
        # Add safety-specific attributes here if needed in the future
        return kwargs

    def to_ros_msg(self) -> msg.RobotStatus:
        """Converts this safety status to a ROS message.

        This method calls the parent's to_ros_msg and would add safety-specific
        data if any existed. Currently safety status shares the same message
        structure as the base class.

        Returns:
            A ROS RobotStatus message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Safety status uses the same message structure as the base class
        # Add safety-specific fields here if needed in the future
        return m


class RobotStatusEvent(Base):
    """Robot status event containing a status message.

    This model represents a discrete event in the robot's status history.
    Each event contains:
    - A timestamp when the event occurred
    - The node that sent the event
    - A reference to the associated status message

    The relationship with RobotStatus is configured with cascade="all, delete"
    to ensure that when a status is deleted, its associated event is also
    removed. The foreign key on status_id uses ondelete="CASCADE" for the
    same purpose.
    """

    __tablename__ = "robot_status_event"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    """Unique identifier for this status event"""

    stamp: Mapped[TimeData] = composite(
        TimeData._generate,
        mapped_column("stamp_nanosec", Integer),
        mapped_column("stamp_sec", Integer),
    )
    """Timestamp when this event was fired.

    Uses a composite column to store the ROS Time message as two separate
    integer columns (nanoseconds and seconds) for database storage.
    """

    sender: Mapped[str] = mapped_column(String(100))
    """Node name that sent this event.

    Identifies which ROS node published the status event.
    """

    status_id: Mapped[int] = mapped_column(
        ForeignKey("robot_status.id", ondelete="CASCADE"),
    )
    """Foreign key to the associated status record.

    This links the event to its corresponding status message. When the
    status is deleted, this event is also deleted (CASCADE).
    """

    status: Mapped["RobotStatus"] = relationship(
        back_populates="event",
        cascade="all, delete",
        single_parent=True,
    )
    """The status message associated with this event.

    This is a bidirectional relationship with the RobotStatus model.
    The cascade configuration ensures that deleting the status also
    deletes this event.
    """

    @classmethod
    def from_ros_msg(cls, m: msg.RobotStatusEvent) -> "RobotStatusEvent":
        """Creates a RobotStatusEvent from a ROS message.

        This method creates a complete status event by first creating
        the associated status (which may be a subclass) and then
        populating the event with the timestamp and sender information.

        Args:
            m: The ROS RobotStatusEvent message to convert

        Returns:
            A RobotStatusEvent instance created from the message
        """
        status = RobotStatus.from_ros_msg(m.status)
        return cls(stamp=TimeData(m.stamp), sender=m.sender, status=status)

    def to_ros_msg(self) -> msg.RobotStatusEvent:
        """Converts this status event to a ROS message.

        This method serializes the event back to a ROS message by
        converting the associated status first, then combining it
        with the event's timestamp and sender information.

        Returns:
            A ROS RobotStatusEvent message populated with this event's data
        """
        status = self.status.to_ros_msg()
        return msg.RobotStatusEvent(stamp=self.stamp.time, sender=self.sender, status=status)
