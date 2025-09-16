"""Contains the Human class

More documentation in the corresponding ros definitions: EntityHuman.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Dict

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column

from .entity import Entity


class Human(Entity):
    """
    A human.
    """

    __tablename__ = "entity_human"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"), primary_key=True
    )

    __mapper_args__ = {
        "polymorphic_identity": "entity_human",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super(cls, cls)._extract_kwargs(m)
        # Assign subclass specific attributes here
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        return m
