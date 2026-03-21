"""Type decorator for byte array data in the ARLab knowledge database.

This module contains the DBInt8Data TypeDecorator for storing byte arrays
in the database. ROS messages often contain binary data (like point cloud
data) that needs to be stored in the database. This decorator handles the
conversion between ROS byte arrays and database storage.

The decorator stores data as signed bytes ('b') for database compatibility,
but converts to unsigned bytes ('B') when reading for ROS compatibility.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

import array

from sqlalchemy import LargeBinary, TypeDecorator


class DBInt8Data(TypeDecorator):
    """Type decorator for storing byte arrays in the database.

    This TypeDecorator handles the conversion between ROS byte arrays
    and database storage. ROS uses unsigned bytes ('B') but the database
    stores them as signed bytes ('b') for compatibility.

    The decorator works with SQLAlchemy's TypeDecorator system to:
    1. Convert ROS byte arrays to signed bytes for database storage
    2. Convert signed bytes back to unsigned bytes when reading from the database

    Usage:
        from arlab_knowledge.db import DBInt8Data
        from sqlalchemy.orm import mapped_column

        data: Mapped[array.array] = mapped_column(DBInt8Data)

    Args:
        impl: The underlying SQLAlchemy type (LargeBinary)
        cache_ok: Whether to cache the type (default True)
    """

    impl = LargeBinary

    def process_bind_param(self, value, dialect):
        """Process the value before storing in the database.

        Converts the byte array to signed bytes for database storage.

        Args:
            value: The byte array value to store
            dialect: The database dialect being used

        Returns:
            The value as signed bytes, or None if value is None
        """
        if value is not None:
            return value.tobytes()
        return value

    def process_result_value(self, value, dialect):
        """Process the value after reading from the database.

        Converts the signed bytes back to an unsigned byte array for
        ROS compatibility.

        Args:
            value: The value read from the database
            dialect: The database dialect being used

        Returns:
            The value as an unsigned byte array, or None if value is None
        """
        if value is not None:
            return array.array("b", value)
        return value
