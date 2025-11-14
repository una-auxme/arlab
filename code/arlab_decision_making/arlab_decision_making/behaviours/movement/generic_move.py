from arlab_common_interfaces.action import MovementAction

from ...utils.constants import MOVEMENT_ACTION_PATH
from ..common.action import RosActionBehaviour


class GenericMovement(RosActionBehaviour):
    def __init__(self, name: str, key: str, result_output_key: str):
        super().__init__(
            name,
            action_type=MovementAction,
            action_name=MOVEMENT_ACTION_PATH,
            key=key,
            result_output_key=result_output_key,
        )