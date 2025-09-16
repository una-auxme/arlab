"""Db adapter for generic ros messages to json

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
"""

import json
from collections.abc import Iterable
from typing import Any, Dict, List, Type

from sqlalchemy import JSON, TypeDecorator

TYPENAME_ENTRY = "__typename__"


def rosmsg2dict(msg) -> Dict | List | Any:
    """Converts msg into a dict or list

    Args:
        msg (_type_): Ros message or list of ros messages

    Returns:
        Dict | List | Any:
            - dict if msg is a ros message
            - list if msg is a list of ros messages
    """
    if hasattr(msg, "get_fields_and_field_types"):
        fields: Dict[str, str] = msg.get_fields_and_field_types()
        return_value = {}
        return_value[TYPENAME_ENTRY] = type(msg).__name__
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
    """Converts msg into json

    Args:
        msg (_type_): Ros message or list of ros messages

    Returns:
        str: json
    """
    field_dict = rosmsg2dict(msg)
    json_str = json.dumps(field_dict)
    return json_str


def dict2rosmsg(msg, d: Dict | List | Any, type_dict: Dict[str, Type]):
    """Converts a dictionary into the given msg

    Args:
        msg (_type_): Target message
        d (Dict | List | Any): Dictionary containing the message data
        type_dict (Dict[str, Type]): Contains class name->python type mappings
            for the ros messages used in msg. This is necessary for sequences

    Raises:
        ValueError: The json does not match msg

    Returns:
        _type_:
            - modified msg if msg is a ros message or empty list
            - d if msg is neither of the above
    """
    if hasattr(msg, "get_fields_and_field_types"):
        if not isinstance(d, Dict):
            raise ValueError(
                f"Unable to convert json: msg {type(msg).__name__} "
                f"does not match json {d}"
            )
        fields: Dict[str, str] = msg.get_fields_and_field_types()
        for field in fields.keys():
            value = getattr(msg, field)
            new_value = dict2rosmsg(value, d[field], type_dict)
            if type(value) is not type(new_value):
                raise ValueError(
                    f"Unable to convert json: field {field} "
                    f"with type {type(value).__name__} does not match "
                    f"json type {type(new_value).__name__}"
                )
            setattr(msg, field, new_value)
    elif isinstance(msg, List) and not isinstance(msg, str):
        if not isinstance(d, List):
            raise ValueError(
                f"Unable to convert json: msg {type(msg).__name__} "
                f"does not match json {d}"
            )
        if len(d) > 0:
            if TYPENAME_ENTRY not in d[0]:
                raise ValueError(
                    f"{TYPENAME_ENTRY} not found in json. "
                    "It is required for sequences/arrays"
                )

            msg_type_name = d[0][TYPENAME_ENTRY]
            if msg_type_name not in type_dict:
                raise ValueError(
                    f"{msg_type_name} not found in type_list. "
                    "It is required for sequences/arrays. Please add it"
                )
            msg_type = type_dict[msg_type_name]
            for item in d:
                msg.append(dict2rosmsg(msg_type(), item, type_dict))
    else:
        msg = d

    return msg


def json2rosmsg(j: str, type_list: List[Type]):
    """Converts j into a ros message

    Args:
        j (str): json
        type_list (List[Type]): List of possible ros message types.
            The conversion will return the type specified in the json.

    Raises:
        ValueError: The json does not match a type in type_list

    Returns:
        _type_: ROS message or list of ROS messages
    """
    field_dict = json.loads(j)
    type_dict = {}
    for t in type_list:
        type_dict[t.__name__] = t
    if isinstance(field_dict, Dict):
        if TYPENAME_ENTRY not in field_dict:
            raise ValueError(f"{TYPENAME_ENTRY} not found in json")
        msg_type_name = field_dict[TYPENAME_ENTRY]
        if msg_type_name not in type_dict:
            raise ValueError(f"{msg_type_name} not found in type_list. Please add it")
        msg_type = type_dict[msg_type_name]
        msg = msg_type()
    elif isinstance(field_dict, List):
        msg = []
    else:
        raise ValueError("Unable to convert json: json is no list or dict")
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
