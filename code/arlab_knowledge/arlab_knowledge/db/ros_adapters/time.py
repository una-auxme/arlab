"""Time adapter class for the ARLab knowledge database.

This module contains the TimeData adapter class for converting ROS Time messages
to database column types and vice versa. These adapters are used with
SQLAlchemy's composite columns to store ROS Time messages directly in
database columns.

The adapter works by:
1. Converting ROS Time messages to flat column values for database storage
2. Reconstructing ROS Time messages when reading from the database

ROS Time messages store time as two separate integer fields:
- sec: Seconds since Unix epoch
- nanosec: Nanoseconds (0 to 999,999,999)

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

from typing import Tuple

from builtin_interfaces.msg import Time


class TimeData:
    """Adapter to create database columns from a ROS Time message.

    This class enables storing a ROS Time message as flat database columns
    using SQLAlchemy's composite column feature. The Time message contains
    two integer fields: sec (seconds) and nanosec (nanoseconds).

    The adapter works by:
    1. Storing the Time message in an instance variable
    2. Providing a _generate classmethod to create instances from column values
    3. Implementing __composite_values__ to extract column values from the Time

    Usage with SQLAlchemy:
        stamp: Mapped[TimeData] = composite(
            TimeData._generate,
            mapped_column("stamp_nanosec", Integer),
            mapped_column("stamp_sec", Integer),
        )

    Args:
        time: The ROS Time message to wrap

    Attributes:
        time: The wrapped ROS Time message
    """

    def __init__(self, time: Time):
        """Initialize the TimeData adapter.

        Args:
            time: The ROS Time message to wrap
        """
        super().__init__()
        self.time = time

    @classmethod
    def _generate(cls, nanosec: int, sec: int) -> "TimeData":
        """Generate a TimeData instance from column values.

        This classmethod creates a TimeData instance and reconstructs
        a ROS Time message from the column values.

        Args:
            nanosec: Nanoseconds (0 to 999,999,999)
            sec: Seconds since Unix epoch

        Returns:
            A TimeData instance with a reconstructed ROS Time message
        """
        time = Time()
        time.nanosec = nanosec
        time.sec = sec
        return TimeData(time)

    def __composite_values__(
        self,
    ) -> Tuple[int, int]:
        """Extract column values from the Time message.

        This method is called by SQLAlchemy to extract the column values
        from the TimeData instance for database storage.

        Returns:
            A tuple of (nanosec, sec) values
        """
        return self.time.nanosec, self.time.sec
