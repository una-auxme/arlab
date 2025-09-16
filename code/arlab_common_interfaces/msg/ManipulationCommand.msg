
string COMMAND_PICK="pick"
string COMMAND_PLACE="place"
string COMMAND_OPEN="open"
string COMMAND_CLOSE="close"
string COMMAND_MOVE="move"
string COMMAND_HOME="home"
# Type of the command to run
string command_type

# Target entity id if applicable (Pick, Place)
int64 target_entityid

# Target pose if applicable (Move)
geometry_msgs/Pose target_pose
