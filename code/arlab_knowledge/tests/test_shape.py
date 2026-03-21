"""Shape tests

Run these tests with `pytest .` or `colcon test`

Test cases:
    Shape class to/from ROS Message Conversion Tests

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from arlab_knowledge import test_utils as utils
from arlab_knowledge.db.entities.shape import Shape


def convert(shape: Shape):
    """Converts shape to and from a ros message.

    Makes sure no data is altered or lost.
    """
    msg = shape.to_ros_msg()
    shape_conv = Shape.from_ros_msg(msg)
    utils.equality_check(shape, shape_conv)


def test_convert_shape():
    shape = utils.get_shape()
    convert(shape)
