# Import all database schema modules so that they are available to the orm
# when the db module is loaded
from . import entity  # noqa: F401
from . import furniture  # noqa: F401
from . import human  # noqa: F401
from . import pickable  # noqa: F401
from . import shape  # noqa: F401
