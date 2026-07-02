"""Entity database schema modules for the ARLab knowledge database.

This submodule contains all SQLAlchemy database models for representing physical
entities around the robot, including:
- Entity: Base class for all entities
- Furniture: Abstract base for furniture objects (tables, cupboards, etc.)
- Human: Human entities
- Pickable: Objects that can be manipulated by the robot
- Shape: Shape information for entities (bounding boxes, point clouds)

The entity type conversion functions enable polymorphic behavior:
- entity_msg_type_to_class: Converts EntityType messages to corresponding classes
- entity_extract_type_msg: Extracts EntityType messages from entity instances

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

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


def entity_msg_type_to_class(msg_type: msg.EntityType) -> type | None:
    """Converts an EntityType message to a corresponding entity class.

    This function enables polymorphic instantiation of entity subclasses
    based on the ROS message type. When a ROS message arrives, this function
    determines which Python class to instantiate.

    Args:
        msg_type: The EntityType message to convert

    Returns:
        The corresponding entity class, or None if the type is not supported
    """
    if msg_type.id == msg.EntityType.ENTITY:
        return entity.Entity
    if msg_type.id == msg.EntityType.PICKABLE:
        return pickable.Pickable
    if msg_type.id == msg.EntityType.HUMAN:
        return human.Human
    if msg_type.id == msg.EntityType.FURNITURE:
        return furniture.Furniture
    if msg_type.id == msg.EntityType.CUPBOARD:
        return furniture.Cupboard
    if msg_type.id == msg.EntityType.DOOR:
        return furniture.Door
    if msg_type.id == msg.EntityType.SHELF:
        return furniture.Shelf
    if msg_type.id == msg.EntityType.TABLE:
        return furniture.Table
    if msg_type.id == msg.EntityType.DISHWASHER:
        return furniture.Dishwasher
    if msg_type.id == msg.EntityType.WASHER:
        return furniture.Washer
    if msg_type.id == msg.EntityType.LAUNDRY_BASKET:
        return furniture.LaundryBasket
    if msg_type.id == msg.EntityType.TRASH_BIN:
        return furniture.TrashBin
    else:
        return None


def entity_extract_type_msg(e: entity.Entity) -> msg.EntityType:
    """Extracts an EntityType message from an entity instance.

    This function enables polymorphic serialization of entities back to ROS
    messages. It determines the appropriate EntityType based on the entity's
    actual class type.

    Args:
        e: The entity instance to extract the type from

    Returns:
        A populated EntityType message with the appropriate id
    """
    m = msg.EntityType()
    m.id = msg.EntityType.ENTITY
    if isinstance(e, human.Human):
        m.id = msg.EntityType.HUMAN
    if isinstance(e, pickable.Pickable):
        m.id = msg.EntityType.PICKABLE
    if isinstance(e, furniture.Furniture):
        m.id = msg.EntityType.FURNITURE
    if isinstance(e, furniture.Cupboard):
        m.id = msg.EntityType.CUPBOARD
    if isinstance(e, furniture.Door):
        m.id = msg.EntityType.DOOR
    if isinstance(e, furniture.Shelf):
        m.id = msg.EntityType.SHELF
    if isinstance(e, furniture.Table):
        m.id = msg.EntityType.TABLE
    if isinstance(e, furniture.Dishwasher):
        m.id = msg.EntityType.DISHWASHER
    if isinstance(e, furniture.Washer):
        m.id = msg.EntityType.WASHER
    if isinstance(e, furniture.LaundryBasket):
        m.id = msg.EntityType.LAUNDRY_BASKET
    if isinstance(e, furniture.TrashBin):
        m.id = msg.EntityType.TRASH_BIN
    return m
