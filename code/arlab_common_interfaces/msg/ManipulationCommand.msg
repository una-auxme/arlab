# Available commands
string COMMAND_PICK="pick"
string COMMAND_PLACE="place"
string COMMAND_OPEN="open"
string COMMAND_CLOSE="close"
string COMMAND_MOVE="move"
string COMMAND_MOVE_TO_BOX="move_to_box"
string COMMAND_HOME="home"
string COMMAND_CYLINDRICAL="cylindrical"
string COMMAND_PINCH="pinch"
string COMMAND_LATERAL="lateral"
string COMMAND_POINTUP="point_up"
string COMMAND_POINTDOWN="point_down"
string COMMAND_SPHERICAL="spherical"
string COMMAND_TRIDIGITAL="tridigital"
string COMMAND_PICK_PINCH="pick_pinch"
string COMMAND_PICK_LATERAL="pick_lateral"
string COMMAND_PICK_SPHERICAL="pick_spherical"
string COMMAND_PICK_TRIDIGITAL="pick_tridigital"

# Type of the command to run
string command_type

# Target entity id if applicable (Pick, Place)
int64 target_entityid

# Target pose if applicable (Move)
geometry_msgs/Pose target_pose
