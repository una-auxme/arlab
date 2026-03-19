# Type of status (e.g., movement, manipulation, safety)
StatusType status_type
# Overall status flag (true if everything is OK)
bool is_ok
# Movement-related status information (valid if status_type == movement)
StatusMovement movement
# Manipulation-related status information (valid if status_type == manipulation)
StatusManipulation manipulation
# Safety-related status information (valid if status_type == safety)
StatusSafety safety
