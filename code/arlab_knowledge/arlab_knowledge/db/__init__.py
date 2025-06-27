# Import all database schema modules so that they are available to the orm
# when the db module is loaded
from . import base  # noqa: F401
from .entities import entity  # noqa: F401
from .entities import furniture  # noqa: F401
from .entities import human  # noqa: F401
from .entities import pickable  # noqa: F401
from .entities import shape  # noqa: F401
