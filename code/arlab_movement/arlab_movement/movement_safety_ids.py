"""Heartbeat IDs of the movement nodes monitored by the movement_local_safety node."""

from enum import IntEnum

class MovementSafetyIDs(IntEnum):
    """
    ID each movement node sends in its heartbeat message to the movement_local_safety node via /arlab/movement/heartbeat.

    Node IDs are unique within the movement module only. On timeout movement_local_safety
    reports the ID as error code in [module_id, error_code] on /global_heartbeat, so
    the CentralSafetyNode reads it together with the movement module_id.
    Node IDs must be > o, as central_safety_node uses error_code > 0 to indicate critical errors.
    """

    MOVEMENT_ORCHESTRATOR = 1
