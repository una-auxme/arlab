"""Entity tests

Run these tests with `pytest .` or `colcon test`

Test cases:
    Entity class to/from ROS Message Conversion Tests

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from arlab_knowledge import test_utils as utils
from arlab_knowledge.db.entities.entity import Entity


def convert(entity: Entity):
    """Converts entity to and from a ros message.

    Makes sure no data is altered or lost.
    """
    msg = entity.to_ros_msg()
    entity_conv = Entity.from_ros_msg(msg)
    utils.equality_check(entity, entity_conv)


def test_convert_entity():
    entity = utils.get_entity()
    convert(entity)


def test_convert_human():
    entity = utils.get_human()
    convert(entity)


def test_convert_pickable():
    entity = utils.get_pickable()
    convert(entity)


def test_convert_furniture():
    entity = utils.get_furniture()
    convert(entity)


def test_convert_cupboard():
    entity = utils.get_cupboard()
    convert(entity)


def test_convert_door():
    entity = utils.get_door()
    convert(entity)


def test_convert_shelf():
    entity = utils.get_shelf()
    convert(entity)


def test_convert_table():
    entity = utils.get_table()
    convert(entity)
