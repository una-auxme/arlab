from typing import List

from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .entity import Entity


class Furniture(Entity):
    __tablename__ = "entity_furniture"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    pickables: Mapped[List["Pickable"]] = relationship(  # type: ignore # noqa: F821
        back_populates="located_on",
        foreign_keys="Pickable.located_on_id",
        cascade="all, delete-orphan",
    )

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture",
    }


class Door(Furniture):
    __tablename__ = "entity_furniture_door"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id"), primary_key=True)

    width: Mapped[float]
    open: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_door",
    }


class Table(Furniture):
    __tablename__ = "entity_furniture_table"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id"), primary_key=True)

    height: Mapped[float]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_table",
    }


class Cupboard(Furniture):
    __tablename__ = "entity_furniture_cupboard"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id"), primary_key=True)

    shelves: Mapped[List["Shelf"]] = relationship(
        back_populates="cupboard",
        foreign_keys="Shelf.cupboard_id",
        cascade="all, delete-orphan",
    )

    width: Mapped[float]
    height: Mapped[float]
    open: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_cupboard",
    }


class Shelf(Furniture):
    __tablename__ = "entity_furniture_shelf"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id"), primary_key=True)

    cupboard_id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture_cupboard.id", ondelete="CASCADE")
    )
    cupboard: Mapped["Cupboard"] = relationship(
        back_populates="shelves", foreign_keys=cupboard_id
    )

    width: Mapped[float]
    height: Mapped[float]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_shelf",
    }
