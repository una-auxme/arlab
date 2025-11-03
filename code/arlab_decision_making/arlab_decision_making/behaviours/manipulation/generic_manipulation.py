from arlab_common_interfaces.action import ManipulationAction

from ...utils.constants import MANIPULATION_ACTION_PATH
from ..common.action import RosActionBehaviour


class GenericManipulation(RosActionBehaviour):
    def __init__(self, name: str, key: str, result_output_key: str):
        super().__init__(
            name,
            action_type=ManipulationAction,
            action_name=MANIPULATION_ACTION_PATH,
            key=key,
            result_output_key=result_output_key,
        )
