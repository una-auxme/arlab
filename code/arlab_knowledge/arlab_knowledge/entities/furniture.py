from . import Entity

from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column


class Furniture(Entity):
    __tablename__ = "furniture"
    id: Mapped[int] = mapped_column(ForeignKey("entity.id"), primary_key=True)

    __mapper_args__ = {
        "polymorphic_identity": "furniture",
    }
