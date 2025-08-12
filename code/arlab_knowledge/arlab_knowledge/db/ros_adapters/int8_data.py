import array

from sqlalchemy import LargeBinary, TypeDecorator


class DBInt8Data(TypeDecorator):
    """Represents a ros message as a db json string

    Usage::

        DBInt8Data(msg)

    """

    impl = LargeBinary

    def process_bind_param(self, value, dialect):
        if value is not None:
            return value.tobytes()
        return value

    def process_result_value(self, value, dialect):
        if value is not None:
            return array.array("b", value)
        return value
