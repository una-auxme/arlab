"""Contains the Pickable class

A Pickable is an object that can by manipulated by the manipulator

More documentation in the corresponding ros definitions: EntityPickable.msg

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

from typing import Dict, Optional

from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey
from sqlalchemy.orm import Mapped, mapped_column, relationship

from .entity import Entity


class Pickable(Entity):
    """
    An object that can by manipulated by the manipulator
    """

    __tablename__ = "entity_pickable"
    id: Mapped[int] = mapped_column(
        ForeignKey("entity.id", ondelete="CASCADE"), primary_key=True
    )

    located_on_id: Mapped[Optional[int]] = mapped_column(
        ForeignKey("entity_furniture.id", ondelete="CASCADE")
    )
    located_on: Mapped[Optional["Furniture"]] = relationship(  # type: ignore # noqa: F821
        back_populates="pickables", foreign_keys=located_on_id
    )

    picking_tag: Mapped[str]

    __mapper_args__ = {
        "polymorphic_identity": "entity_pickable",
    }

    @classmethod
    def _extract_kwargs(cls, m: msg.Entity) -> Dict:
        kwargs = super()._extract_kwargs(m)
        # Assign subclass specific attributes here
        kwargs["picking_tag"] = m.pickable.picking_tag
        return kwargs

    def to_ros_msg(self) -> msg.Entity:
        m = super().to_ros_msg()
        # Assign subclass specific attributes here
        m.pickable.picking_tag = self.picking_tag
        return m
