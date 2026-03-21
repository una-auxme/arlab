"""Status tests

Run these tests with `pytest .` or `colcon test`

Test cases:
    Status class to/from ROS Message Conversion Tests

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
    Daniel Gabler <daniel.gabler@uni-a.de>
"""

from arlab_knowledge import test_utils as utils
from arlab_knowledge.db.status import RobotStatus, RobotStatusEvent


def convert(status: RobotStatus):
    """Converts status to and from a ros message.

    Makes sure no data is altered or lost.
    """
    msg = status.to_ros_msg()
    status_conv = RobotStatus.from_ros_msg(msg)
    utils.equality_check(status, status_conv)


def test_convert_robot_status_event():
    event = utils.get_robot_status_event()
    msg = event.to_ros_msg()
    event_conv = RobotStatusEvent.from_ros_msg(msg)
    utils.equality_check(event, event_conv)


def test_convert_robot_status():
    status = utils.get_robot_status()
    convert(status)


def test_convert_manipulation_status():
    status = utils.get_manipulation_status()
    convert(status)


def test_convert_movement_status():
    status = utils.get_movement_status()
    convert(status)


def test_convert_safety_status():
    status = utils.get_safety_status()
    convert(status)
