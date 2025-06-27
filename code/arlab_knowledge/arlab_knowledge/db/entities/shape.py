from sqlalchemy import ForeignKey
from sqlalchemy.orm import (
    Mapped,
    mapped_column,
    relationship,
)

from ..base import Base


class Shape(Base):
    __tablename__ = "shape"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)

    entity_id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"), unique=True
    )
    entity: Mapped["Entity"] = relationship(back_populates="shape", single_parent=True)  # type: ignore # noqa: F821
