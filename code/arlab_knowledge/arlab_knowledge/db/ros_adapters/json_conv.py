import json
from collections.abc import Iterable
from typing import Any, Dict, List, Type

from sqlalchemy import JSON, TypeDecorator


def rosmsg2dict(msg) -> Dict | List | Any:
    if hasattr(msg, "get_fields_and_field_types"):
        fields: Dict[str, str] = msg.get_fields_and_field_types()
        return_value = {}
        return_value["__typename__"] = type(msg).__name__
        for field in fields.keys():
            value = getattr(msg, field)
            return_value[field] = rosmsg2dict(value)
    elif isinstance(msg, Iterable) and not isinstance(msg, str):
        return_value = []
        for item in msg:
            return_value.append(rosmsg2dict(item))
    else:
        return_value = msg

    return return_value


def rosmsg2json(msg) -> str:
    field_dict = rosmsg2dict(msg)
    json_str = json.dumps(field_dict)
    return json_str


def dict2rosmsg(msg, d: Dict | List | Any, type_dict: Dict[str, Type]):
    """Convert a dictionary into the given message

    Args:
        msg (_type_): _description_
        d (Dict | List | Any): _description_
        type_dict (Dict[str, Type]): Contains class name->python type mappings
            for the ros messages used in msg. This is necessary for sequences

    Returns:
        _type_: _description_
    """
    if hasattr(msg, "get_fields_and_field_types"):
        assert isinstance(d, Dict)
        fields: Dict[str, str] = msg.get_fields_and_field_types()
        for field in fields.keys():
            value = getattr(msg, field)
            setattr(msg, field, dict2rosmsg(value, d[field], type_dict))
    elif isinstance(msg, List) and not isinstance(msg, str):
        assert isinstance(d, List)
        if len(d) > 0:
            msg_type_name = d[0]["__typename__"]
            msg_type = type_dict[msg_type_name]
            for item in d:
                msg.append(dict2rosmsg(msg_type(), item, type_dict))
    else:
        msg = d

    return msg


def json2rosmsg(j: str, type_list: List[Type]):
    field_dict = json.loads(j)
    type_dict = {}
    for t in type_list:
        type_dict[t.__name__] = t
    if isinstance(field_dict, Dict):
        msg_type_name = field_dict["__typename__"]
        msg_type = type_list[msg_type_name]
        msg = msg_type()
    elif isinstance(field_dict, List):
        msg = []
    else:
        raise ValueError("Unable to convert json")
    return dict2rosmsg(msg, field_dict, type_dict)


class DBRosMsgJson(TypeDecorator):
    """Represents a ros message as a db json string

    Usage::

        DBRosMsgJson(msg)

    """

    impl = JSON

    def __init__(self, type_list: List[Type], *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.msg_type_list = type_list

    def process_bind_param(self, value, dialect):
        if value is not None:
            return rosmsg2json(value)
        return value

    def process_result_value(self, value, dialect):
        if value is not None:
            return json2rosmsg(value, self.msg_type_list)
        return value
