"""ROS message adapter classes for the ARLab knowledge database.

This submodule contains adapter classes that convert ROS messages to database
column types and vice versa. These adapters are used with SQLAlchemy's
composite columns to store ROS message data directly in database columns.

The adapters work by:
1. Converting ROS messages to flat column values for database storage
2. Reconstructing ROS messages when reading from the database

Adapter classes:
- TimeData: Adapter for builtin_interfaces.Time messages
- PoseData: Adapter for geometry_msgs.Pose messages
- Pose2DData: Adapter for vision_msgs.Pose2D messages
- OccupancyGridData: Adapter for nav_msgs.OccupancyGrid messages
- DBInt8Data: Type decorator for byte array data
- DBRosMsgJson: JSON converter for generic ROS messages

Maintainers:
    Peter Viechter <peter.viechter@uni-a.de>
"""
