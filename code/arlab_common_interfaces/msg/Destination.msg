
int32 ENTITY=0
int32 POSE=1

# Enum-like type of a destination. One of the constants defined above.
int32 destination_type

# Id of the destination entity if destination_type==ENTITY
int64 entityid
# Destination pose if destination_type==POSE
geometry_msgs/Pose pose
