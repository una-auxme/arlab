# Enum-like states of the safety chain (arlab_safety). Constants only, not sent as a message.
#
# A local safety node sends [module_id, state] as std_msgs/Int32MultiArray on /global_heartbeat.
# The CentralSafetyNode reads state as follows:
#   -1          module is registering
#    0          module is OK
#   100-999     node timeout, state is the node ID from SafetyNode
#   1000-1999   module error found by the checks of a local safety node
#
# Module errors use the range 1000 + module_id * 100 to 1000 + module_id * 100 + 99,
# with module_id from SafetyModule.

int32 REGISTERING=-1
int32 OK=0

# Movement (SafetyModule.MOVEMENT): 1100-1199
# SLAM Toolbox stopped or is not active while the movement orchestrator expects it to run
int32 MOVEMENT_SLAM_FAILED=1100
# NAV2 stopped or its lifecycle manager reports the stack inactive while expected to run
int32 MOVEMENT_NAV2_FAILED=1101

# Manipulation (SafetyModule.MANIPULATION): 1200-1299
