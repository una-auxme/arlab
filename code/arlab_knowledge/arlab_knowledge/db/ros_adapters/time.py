from typing import Tuple

from builtin_interfaces.msg import Time


class TimeData:
    """Adapter to create database columns from a ROS Time"""

    def __init__(self, time: Time):
        super().__init__()
        self.time = time

    @classmethod
    def _generate(cls, nanosec: int, sec: int) -> "TimeData":
        """Generate a Time from a row"""
        time = Time()
        time.nanosec = nanosec
        time.sec = sec
        return TimeData(time)

    def __composite_values__(
        self,
    ) -> Tuple[int, int]:
        return self.time.nanosec, self.time.sec
