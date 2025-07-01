from typing import Optional

from .entity import Entity

from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship


class Pickable(Entity):
    __tablename__ = "entity_pickable"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    located_on_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE")
    )
    located_on: Mapped[Optional["Furniture"]] = relationship(  # type: ignore # noqa: F821
        back_populates="pickables"
    )

    max_picking_force: Mapped[float]

    __mapper_args__ = {
        "polymorphic_identity": "entity_pickable",
    }
