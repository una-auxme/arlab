# Import all database schema modules so that they are available to the orm
# when the db module is loaded
from . import (
    base,  # noqa: F401
    entities,  # noqa: F401
    map,  # noqa: F401
    status,  # noqa: F401
)

DB_LOGGER_NAME = "knowledge_database"
