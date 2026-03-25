"""Database schema modules for the ARLab knowledge database.

This package contains all SQLAlchemy database models for storing robot knowledge
including maps, entities (furniture, humans, pickables), shapes, and robot status.

The database schema is organized into the following submodules:
- base: Base class for all database tables using SQLAlchemy's DeclarativeBase
- entities: Entity classes representing physical objects around the robot
- map: Map database schema based on ROS OccupancyGrid messages
- status: Robot status and status event database schemata
- ros_adapters: Adapter classes for converting ROS messages to database columns

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

# Import all database schema modules so that they are available to the orm
# when the db module is loaded
from . import (
    base,  # noqa: F401
    entities,  # noqa: F401
    map,  # noqa: F401
    status,  # noqa: F401
)

DB_LOGGER_NAME = "knowledge_database"
