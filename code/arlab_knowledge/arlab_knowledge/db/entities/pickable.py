from .entity import Entity

from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column


class Pickable(Entity):
    __tablename__ = "pickable"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    __mapper_args__ = {
        "polymorphic_identity": "pickable",
    }
