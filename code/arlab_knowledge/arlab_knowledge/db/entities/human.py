"""Human database schema for the ARLab knowledge database.

This module contains the Human class for representing humans around the
robot. Humans are a special type of entity that may have additional attributes
in the future if needed.

More documentation in the corresponding ROS definitions: EntityHuman.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Dict

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column

from .entity import Entity


class Human(Entity):
    """A human entity.

    This subclass represents a human around the robot. Currently, humans
    share the same attributes as the base Entity class (position, description,
    etc.). Additional human-specific attributes could be added in the future
    if needed (e.g., name, age, role).
    """

    __tablename__ = "entity_human"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent entity record.

    This column serves as the primary key for the human table while also
    linking to the parent Entity record. When the parent entity is deleted,
    this human record is also deleted (CASCADE).
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity_human",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Human instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then would add human-specific attributes if any existed. Currently
        humans use the same attributes as the base Entity class.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent Entity class
        """
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Human uses the same attributes as the base Entity class
        # Add human-specific attributes here if needed in the future
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this human instance to a ROS message.

        This method calls the parent's to_ros_msg and would add human-specific
        data if any existed. Currently humans use the same message structure
        as the base Entity class.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Human uses the same message structure as the base Entity class
        # Add human-specific fields here if needed in the future
        return m
