# Mia Hand planning fix

MoveIt aborted hand planning with `START_STATE_INVALID` because Gazebo/ros2_control reported `j_index_fle` as a tiny negative value around `-2e-08`, while the lower MoveIt limit was exactly `0.0`.

Changes made:

- Relaxed MIA flexion lower limits from `0.0` to `-0.001` in `manipulator_ur_moveit_config/config/joint_limits.yaml`.
- Relaxed the same simulated ros2_control command minima in `manipulator_description/urdf/mia_hand.ros2_control.xacro`.
- Set the `hand_open` named state flexion joints to `0.01` instead of exactly `0.0`, so MoveIt does not start or command directly on the lower boundary.

This is only a numerical tolerance for simulation and does not change the intended open/close behavior of the hand.
