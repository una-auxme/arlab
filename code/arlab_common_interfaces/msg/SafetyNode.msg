# Enum-like node IDs of the safety chain (arlab_safety). Constants only, not sent as a message.
#
# Each monitored node sends its ID as std_msgs/Int32 on the heartbeat topic of its module,
# e.g. /arlab/movement/heartbeat. If a node times out, the local safety node of the module
# reports the node ID as state on /global_heartbeat, see SafetyState.
#
# IDs are unique project-wide. Each module owns the range module_id * 100 to module_id * 100 + 99,
# with module_id from SafetyModule. All IDs are > 0, because the CentralSafetyNode reads
# every state > 0 as critical error.

# Movement (SafetyModule.MOVEMENT): 100-199
int32 MOVEMENT_ORCHESTRATOR=100

# Manipulation (SafetyModule.MANIPULATION): 200-299
