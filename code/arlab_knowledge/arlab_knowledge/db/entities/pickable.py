"""Pickable database schema for the ARLab knowledge database.

This module contains the Pickable class for representing objects that can be
manipulated by the robot's manipulator. Pickables are entities with additional
attributes for object identification (name and category).

More documentation in the corresponding ROS definitions: EntityPickable.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Dict, Optional

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .entity import Entity


class Pickable(Entity):
    """An object that can be manipulated by the manipulator.

    This subclass represents objects that the robot can pick up, move, or
    manipulate. Each pickable has:
    - A unique object name for identification
    - An object category for grouping similar objects
    - An optional reference to the furniture it's located on

    Pickables can be located on furniture (tables, shelves, etc.) or on the
    ground. The located_on relationship is optional and uses a foreign key
    to the furniture table.
    """

    __tablename__ = "entity_pickable"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent entity record.

    This column serves as the primary key for the pickable table while also
    linking to the parent Entity record. When the parent entity is deleted,
    this pickable record is also deleted (CASCADE).
    """

    located_on_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE"),
    )
    """Foreign key to the furniture this pickable is located on.

    This is an optional foreign key that links the pickable to a furniture
    object (table, shelf, etc.). When the furniture is deleted, this pickable
    is also deleted (CASCADE). If None, the pickable is on the ground.
    """

    located_on: Mapped[Optional["Furniture"]] = relationship(  # type: ignore # noqa: F821
        back_populates="pickables",
        foreign_keys=located_on_id,
    )
    """Furniture this pickable is located on.

    This is a bidirectional relationship with the Furniture model.
    """

    object_name: Mapped[str]
    """Name of the object.

    This is a human-readable identifier for the pickable object (e.g., "beer",
    "apple", "banana").

    Should be one of the names defined in EntityPickable.msg.
    For more free-form descriptions/names use the general entity description.
    """

    object_category: Mapped[str]
    """Category of the object.

    This categorizes objects based on their shape (e.g., "sphere", "cube", "cylinder").

    Should be one of the categories defined in EntityPickable.msg.
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity_pickable",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Pickable instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then extracts pickable-specific attributes from the ROS message.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent class plus
            object_name and object_category from the pickable-specific message fields
        """
        kwargs = super()._extract_kwargs(m)
        # Extract pickable-specific attributes from the nested message structure
        kwargs["object_name"] = m.pickable.object_name
        kwargs["object_category"] = m.pickable.object_category
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this pickable instance to a ROS message.

        This method calls the parent's to_ros_msg and populates the pickable-specific
        fields in the nested message structure.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Populate pickable-specific fields in the nested message structure
        m.pickable.object_name = self.object_name
        m.pickable.object_category = self.object_category
        return m
