from typing import Optional
import numpy as np
from io import BytesIO
from sqlalchemy import LargeBinary, TypeDecorator

from nav_msgs.msg import OccupancyGrid


class DBOccupancygridData(TypeDecorator):
    """Represents a ros OccupancyGrid as a db BLOB

    Usage::

        DBOccupancygridData(msg)

    """

    impl = LargeBinary

    def process_bind_param(
        self, value: Optional[OccupancyGrid], dialect
    ) -> Optional[bytes]:
        if value is not None:
            np_grid = np.array(value.data, dtype=np.int8)
            with BytesIO() as buffer:
                np.save(buffer, np_grid, allow_pickle=False)
                byte_data = buffer.getvalue()
        else:
            byte_data = None
        return byte_data

    def process_result_value(
        self, value: Optional[bytes], dialect
    ) -> Optional[OccupancyGrid]:
        if value is not None:
            with BytesIO() as buffer:
                buffer.write(value)
                np_grid = np.load(buffer, allow_pickle=False)
            oc_msg = OccupancyGrid()
            oc_msg.data = np_grid.tolist()
        else:
            oc_msg = None
        return oc_msg
