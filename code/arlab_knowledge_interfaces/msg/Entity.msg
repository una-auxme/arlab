# Type of entity (e.g., furniture, human, pickable, etc.)
EntityType entity_type
# Time stamp of the last update
builtin_interfaces/Time stamp
# Human-readable description (mainly for Human-Robot Interaction)
string description
# Pose of the entity relative to the pose_reference_frame
geometry_msgs/Pose pose
# Reference frame for the pose (e.g., "map", "base_link")
string pose_reference_frame
# Furniture-specific attributes (valid if entity_type == furniture)
EntityFurniture furniture
# Human-specific attributes (valid if entity_type == human)
EntityHuman human
# Pickable object attributes (valid if entity_type == pickable)
EntityPickable pickable

