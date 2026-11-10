# Enum-like module IDs of the safety chain (arlab_safety). Constants only, not sent as a message.
#
# Each module runs one local safety node, which sends [module_id, state] as
# std_msgs/Int32MultiArray on /global_heartbeat to the CentralSafetyNode.
# module_id is one of the constants below and must be unique project-wide.
#
# The module ID also sets the number ranges of the module in SafetyNode
# (module_id * 100 + n) and SafetyState (1000 + module_id * 100 + n).

int32 MOVEMENT=1
int32 MANIPULATION=2
