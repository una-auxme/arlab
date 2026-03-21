"""Contains the base class for all database tables.

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""

# https://docs.sqlalchemy.org/en/20/orm/extensions/asyncio.html#preventing-implicit-io-when-using-asyncsession
from sqlalchemy.ext.asyncio import AsyncAttrs
from sqlalchemy.orm import DeclarativeBase


class Base(AsyncAttrs, DeclarativeBase):
    """Base for the database schema"""

    pass
