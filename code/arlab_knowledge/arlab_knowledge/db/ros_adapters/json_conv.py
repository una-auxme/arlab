import json
from typing import Dict

from sqlalchemy import JSON, TypeDecorator


def rosmsg2dict(msg) -> Dict:
    fields: Dict[str, str] = msg.get_fields_and_field_types()
    field_dict = {}

    for field in fields.keys():
        value = getattr(msg, field)
        if hasattr(value, "get_fields_and_field_types"):
            field_dict[field] = rosmsg2dict(value)
        else:
            field_dict[field] = value
    return field_dict


def rosmsg2json(msg) -> str:
    field_dict = rosmsg2dict(msg)
    json_str = json.dumps(field_dict)
    return json_str


def dict2rosmsg(msg, d: Dict):
    fields: Dict[str, str] = msg.get_fields_and_field_types()
    for field in fields.keys():
        value = getattr(msg, field)
        if hasattr(value, "get_fields_and_field_types"):
            setattr(msg, field, dict2rosmsg(value, d[field]))
        else:
            setattr(msg, field, d[field])
    return msg


def json2rosmsg(msg, j: str):
    field_dict = json.loads(j)
    return dict2rosmsg(msg, field_dict)


class DBRosMsgJson(TypeDecorator):
    """Represents a ros message as a db json string

    Usage::

        DBRosMsgJson(msg)

    """

    impl = JSON

    def process_bind_param(self, value, dialect):
        if value is not None:
            return rosmsg2json(value)
        return value

    def process_result_value(self, value, dialect):
        msg = self.python_type()
        if value is not None:
            return json2rosmsg(msg, value)
        return value
