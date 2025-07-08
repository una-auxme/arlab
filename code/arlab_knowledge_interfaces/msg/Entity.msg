# (Sub)class type
EntityType entity_type
# Time stamp when the entity was last updated
builtin_interfaces/Time stamp
# Human-readable description. mostly used for the HRI
string description
# Pose inside the pose_reference_frame
geometry_msgs/Pose pose
string pose_reference_frame

# Subclass attributes
EntityFurniture furniture
EntityHuman human
EntityPickable pickable
