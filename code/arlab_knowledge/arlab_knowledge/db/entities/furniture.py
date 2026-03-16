"""Contains the Furniture class and subclasses

A Furniture is an Entity that can contain Pickables

More documentation in the corresponding ros definitions: EntityFurniture.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Dict, List

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .entity import Entity


class Furniture(Entity):
    """
    A Furniture is an Entity that can contain Pickables
    """

    __tablename__ = "entity_furniture"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id", ondelete="CASCADE"), primary_key=True)

    pickables: Mapped[List["Pickable"]] = relationship(  # type: ignore # noqa: F821
        back_populates="located_on",
        foreign_keys="Pickable.located_on_id",
        cascade="all, delete-orphan",
    )
    """Pickables located on this Furniture"""

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        return m


class Cupboard(Furniture):
    """
    A Cupboard with shelves
    """

    __tablename__ = "entity_furniture_cupboard"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id", ondelete="CASCADE"), primary_key=True)

    shelves: Mapped[List["Shelf"]] = relationship(
        back_populates="cupboard",
        foreign_keys="Shelf.cupboard_id",
        cascade="all, delete-orphan",
    )
    """Shelves that are part of this cupboard"""

    width: Mapped[float]
    height: Mapped[float]
    open: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_cupboard",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        kwargs["width"] = m.furniture.cupboard.width
        kwargs["height"] = m.furniture.cupboard.height
        kwargs["open"] = m.furniture.cupboard.open
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        m.furniture.cupboard.width = self.width
        m.furniture.cupboard.height = self.height
        m.furniture.cupboard.open = self.open
        return m


class Door(Furniture):
    """
    A door.
    """

    __tablename__ = "entity_furniture_door"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id", ondelete="CASCADE"), primary_key=True)

    width: Mapped[float]
    open: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_door",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        kwargs["width"] = m.furniture.door.width
        kwargs["open"] = m.furniture.door.open
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        m.furniture.door.width = self.width
        m.furniture.door.open = self.open
        return m


class Shelf(Furniture):
    """
    A shelf that is part of a cupboard
    """

    __tablename__ = "entity_furniture_shelf"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id", ondelete="CASCADE"), primary_key=True)

    cupboard_id: Mapped[int] = mapped_column(ForeignKey("entity_furniture_cupboard.id", ondelete="CASCADE"))
    cupboard: Mapped["Cupboard"] = relationship(back_populates="shelves", foreign_keys=cupboard_id)
    """The cupboard this shelf belongs to"""

    width: Mapped[float]
    height: Mapped[float]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_shelf",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        kwargs["cupboard_id"] = m.furniture.shelf.cupboard_id
        kwargs["width"] = m.furniture.shelf.width
        kwargs["height"] = m.furniture.shelf.height
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        m.furniture.shelf.cupboard_id = self.cupboard_id
        m.furniture.shelf.width = self.width
        m.furniture.shelf.height = self.height
        return m


class Table(Furniture):
    """
    A table.
    """

    __tablename__ = "entity_furniture_table"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id", ondelete="CASCADE"), primary_key=True)

    height: Mapped[float]
    """Table height above the ground"""

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_table",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        kwargs["height"] = m.furniture.table.height
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        m.furniture.table.height = self.height
        return m
