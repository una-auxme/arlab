from typing import List, Optional
from sqlalchemy import ForeignKey, String
from sqlalchemy.orm import (
    DeclarativeBase,
    Mapped,
    mapped_column,
    relationship,
    composite,
)

from arlab_knowledge import Base

import arlab_knowledge.entities.entity as entity_module


class Shape(Base):
    __tablename__ = "shape"

    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    entity_id: Mapped[int] = mapped_column(ForeignKey("entity.id"), unique=True)

    entity: Mapped["entity_module.Entity"] = relationship(
        back_populates="shape", single_parent=True
    )
