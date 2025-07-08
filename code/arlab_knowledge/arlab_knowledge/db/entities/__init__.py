from arlab_knowledge_interfaces import msg

# Import all database schema modules so that they are available to the orm
# when the db module is loaded
from . import (
    entity,  # noqa: F401
    furniture,  # noqa: F401
    human,  # noqa: F401
    pickable,  # noqa: F401
    shape,  # noqa: F401
)


def entity_msg_type_to_class(msg_type: msg.EntityType):
    if msg_type.entity_type == msg.EntityType.ENTITY:
        return entity.Entity
    if msg_type.entity_type == msg.EntityType.PICKABLE:
        return pickable.Pickable
    if msg_type.entity_type == msg.EntityType.HUMAN:
        return human.Human
    if msg_type.entity_type == msg.EntityType.FURNITURE:
        return furniture.Furniture
    if msg_type.entity_type == msg.EntityType.CUPBOARD:
        return furniture.Cupboard
    if msg_type.entity_type == msg.EntityType.DOOR:
        return furniture.Door
    if msg_type.entity_type == msg.EntityType.SHELF:
        return furniture.Shelf
    if msg_type.entity_type == msg.EntityType.TABLE:
        return furniture.Table
    else:
        return None


def entity_extract_type_msg(e: entity.Entity) -> msg.EntityType:
    m = msg.EntityType()
    m.entity_type = msg.EntityType.ENTITY
    if isinstance(e, human.Human):
        m.entity_type = msg.EntityType.HUMAN
    if isinstance(e, pickable.Pickable):
        m.entity_type = msg.EntityType.PICKABLE
    if isinstance(e, furniture.Furniture):
        m.entity_type = msg.EntityType.FURNITURE
    if isinstance(e, furniture.Cupboard):
        m.entity_type = msg.EntityType.CUPBOARD
    if isinstance(e, furniture.Door):
        m.entity_type = msg.EntityType.DOOR
    if isinstance(e, furniture.Shelf):
        m.entity_type = msg.EntityType.SHELF
    if isinstance(e, furniture.Table):
        m.entity_type = msg.EntityType.TABLE
    return m
