"""Furniture database schema for the ARLab knowledge database.

This module contains the Furniture base class and its subclasses for representing
different types of furniture objects around the robot. Furniture objects can
contain pickables (objects that can be manipulated) and have specific properties
depending on their type.

Furniture types:
- Furniture: Abstract base class for all furniture
- Cupboard: A cupboard with shelves
- Door: A door
- Shelf: A shelf that is part of a cupboard
- Table: A table

The furniture system uses polymorphic inheritance to handle different furniture
types while maintaining a unified database schema. Each furniture type has
specific attributes stored in separate tables.

More documentation in the corresponding ROS definitions: EntityFurniture.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Dict, List

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .entity import Entity


class Furniture(Entity):
    """Furniture is an Entity that can contain Pickables.

    This is an abstract base class for all furniture objects. Furniture objects
    represent physical objects that can hold or contain other objects (pickables).
    Examples include tables, cupboards, shelves, etc.

    The relationship with Pickables allows furniture to hold multiple objects,
    with cascade="all, delete-orphan" ensuring that when furniture is deleted,
    all its pickables are also deleted.
    """

    __tablename__ = "entity_furniture"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent entity record.

    This column serves as the primary key for the furniture table while also
    linking to the parent Entity record. When the parent entity is deleted,
    this furniture record is also deleted (CASCADE).
    """

    pickables: Mapped[List["Pickable"]] = relationship(  # type: ignore # noqa: F821
        back_populates="located_on",
        foreign_keys="Pickable.located_on_id",
        cascade="all, delete-orphan",
    )
    """Pickables located on this furniture.

    This relationship enables furniture to hold multiple pickable objects.
    The cascade configuration ensures that when furniture is deleted, all
    its pickables are also deleted.
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Furniture instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then would add furniture-specific attributes if any existed. Currently
        furniture uses the same attributes as the base Entity class.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent Entity class
        """
        kwargs = super()._extract_kwargs(m)
        # Furniture uses the same attributes as the base Entity class
        # Add furniture-specific attributes here if needed in the future
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this furniture instance to a ROS message.

        This method calls the parent's to_ros_msg and would add furniture-specific
        data if any existed. Currently furniture uses the same message structure
        as the base Entity class.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Furniture uses the same message structure as the base Entity class
        # Add furniture-specific fields here if needed in the future
        return m


class Cupboard(Furniture):
    """A cupboard with shelves.

    This subclass represents a cupboard furniture object that contains shelves.
    Cupboards have specific dimensions (width, height) and an open/closed state.
    They can contain multiple shelves, which are stored in separate database
    records linked to the cupboard.
    """

    __tablename__ = "entity_furniture_cupboard"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent furniture record.

    This column serves as the primary key for the cupboard table while also
    linking to the parent Furniture record. When the parent is deleted,
    this cupboard record is also deleted (CASCADE).
    """

    shelves: Mapped[List["Shelf"]] = relationship(
        back_populates="cupboard",
        foreign_keys="Shelf.cupboard_id",
        cascade="all, delete-orphan",
    )
    """Shelves that are part of this cupboard.

    This relationship enables a cupboard to contain multiple shelves.
    The cascade configuration ensures that when the cupboard is deleted,
    all its shelves are also deleted.
    """

    width: Mapped[float]
    """Width of the cupboard in meters."""

    height: Mapped[float]
    """Height of the cupboard in meters."""

    open: Mapped[str]
    """Open state of the cupboard.

    Stores the open/closed state as a string (e.g., "open", "closed", "partially_open").
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_cupboard",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Cupboard instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then extracts cupboard-specific attributes from the ROS message.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent class plus
            width, height, and open from the cupboard-specific message fields
        """
        kwargs = super()._extract_kwargs(m)
        # Extract cupboard-specific attributes from the nested message structure
        kwargs["width"] = m.furniture.cupboard.width
        kwargs["height"] = m.furniture.cupboard.height
        kwargs["open"] = m.furniture.cupboard.open
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this cupboard instance to a ROS message.

        This method calls the parent's to_ros_msg and populates the cupboard-specific
        fields in the nested message structure.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Populate cupboard-specific fields in the nested message structure
        m.furniture.cupboard.width = self.width
        m.furniture.cupboard.height = self.height
        m.furniture.cupboard.open = self.open
        return m


class Door(Furniture):
    """A door.

    This subclass represents a door furniture object. Doors have specific
    dimensions (width) and an open/closed state.
    """

    __tablename__ = "entity_furniture_door"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent furniture record.

    This column serves as the primary key for the door table while also
    linking to the parent Furniture record. When the parent is deleted,
    this door record is also deleted (CASCADE).
    """

    width: Mapped[float]
    """Width of the door in meters."""

    open: Mapped[str]
    """Open state of the door.

    Stores the open/closed state as a string (e.g., "open", "closed").
    """

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_door",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Door instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then extracts door-specific attributes from the ROS message.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent class plus
            width and open from the door-specific message fields
        """
        kwargs = super()._extract_kwargs(m)
        # Extract door-specific attributes from the nested message structure
        kwargs["width"] = m.furniture.door.width
        kwargs["open"] = m.furniture.door.open
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this door instance to a ROS message.

        This method calls the parent's to_ros_msg and populates the door-specific
        fields in the nested message structure.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Populate door-specific fields in the nested message structure
        m.furniture.door.width = self.width
        m.furniture.door.open = self.open
        return m


class Shelf(Furniture):
    """A shelf that is part of a cupboard.

    This subclass represents a shelf that belongs to a cupboard. Shelves have
    specific dimensions (width, height) and are linked to their parent cupboard
    through a foreign key relationship.
    """

    __tablename__ = "entity_furniture_shelf"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent furniture record.

    This column serves as the primary key for the shelf table while also
    linking to the parent Furniture record. When the parent is deleted,
    this shelf record is also deleted (CASCADE).
    """

    cupboard_id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture_cupboard.id", ondelete="CASCADE"),
    )
    """Foreign key to the parent cupboard.

    This links the shelf to its parent cupboard. When the cupboard is deleted,
    this shelf is also deleted (CASCADE).
    """

    cupboard: Mapped["Cupboard"] = relationship(
        back_populates="shelves",
        foreign_keys=cupboard_id,
    )
    """The cupboard this shelf belongs to.

    This is a bidirectional relationship with the Cupboard model.
    """

    width: Mapped[float]
    """Width of the shelf in meters."""

    height: Mapped[float]
    """Height of the shelf in meters."""

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_shelf",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Shelf instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then extracts shelf-specific attributes from the ROS message.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent class plus
            cupboard_id, width, and height from the shelf-specific message fields
        """
        kwargs = super()._extract_kwargs(m)
        # Extract shelf-specific attributes from the nested message structure
        kwargs["cupboard_id"] = m.furniture.shelf.cupboard_id
        kwargs["width"] = m.furniture.shelf.width
        kwargs["height"] = m.furniture.shelf.height
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this shelf instance to a ROS message.

        This method calls the parent's to_ros_msg and populates the shelf-specific
        fields in the nested message structure.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Populate shelf-specific fields in the nested message structure
        m.furniture.shelf.cupboard_id = self.cupboard_id
        m.furniture.shelf.width = self.width
        m.furniture.shelf.height = self.height
        return m


class Table(Furniture):
    """A table.

    This subclass represents a table furniture object. Tables have a specific
    height attribute that indicates the table height above the ground.
    """

    __tablename__ = "entity_furniture_table"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE"),
        primary_key=True,
    )
    """Foreign key to the parent furniture record.

    This column serves as the primary key for the table table while also
    linking to the parent Furniture record. When the parent is deleted,
    this table record is also deleted (CASCADE).
    """

    height: Mapped[float]
    """Table height above the ground in meters."""

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_table",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        """Extracts keyword arguments for creating a Table instance.

        This method calls the parent's _extract_kwargs to get common attributes,
        then extracts table-specific attributes from the ROS message.

        Args:
            m: The ROS Entity message to extract data from

        Returns:
            A dictionary containing attributes from the parent class plus
            height from the table-specific message field
        """
        kwargs = super()._extract_kwargs(m)
        # Extract table-specific attributes from the nested message structure
        kwargs["height"] = m.furniture.table.height
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        """Converts this table instance to a ROS message.

        This method calls the parent's to_ros_msg and populates the table-specific
        field in the nested message structure.

        Returns:
            A ROS Entity message populated with this instance's data
        """
        m = super().to_ros_msg()
        # Populate table-specific field in the nested message structure
        m.furniture.table.height = self.height
        return m
