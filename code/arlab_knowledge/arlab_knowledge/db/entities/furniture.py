from typing import List

from .entity import Entity

from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship


class Furniture(Entity):
    __tablename__ = "entity_furniture"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    pickables: Mapped[List["Pickable"]] = relationship(  # type: ignore # noqa: F821
        back_populates="located_on",
        cascade="all, delete",
        passive_deletes=True,
    )

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture",
    }


class Door(Furniture):
    __tablename__ = "entity_furniture_door"
    id: Mapped[int] = mapped_column(ForeignKey("entity_furniture.id"), primary_key=True)

    width: Mapped[float]
    is_open: Mapped[bool]

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
        cascade="all, delete",
        passive_deletes=True,
    )

    width: Mapped[float]
    height: Mapped[float]
    is_open: Mapped[bool]

    __mapper_args__ = {
        "polymorphic_identity": "entity_furniture_cupboard",
    }


class Shelf(Entity):
    __tablename__ = "entity_shelf"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    cupboard_id: Mapped[int] = mapped_column(
        ForeignKey("entity_furniture_cupboard.id", ondelete="CASCADE")
    )
    cupboard: Mapped["Cupboard"] = relationship(back_populates="shelves")

    height: Mapped[float]

    __mapper_args__ = {
        "polymorphic_identity": "entity_shelf",
    }
