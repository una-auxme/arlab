# ARLAB Manipulator MoveIt Configuration

Customized MoveIt 2 configuration based on the official [`ur_moveit_config`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) package for the `ARLab` at the University of Augsburg in the context of the `Zirbi` robot.

**Maintainer:** Sofia Öttl <sofia.oettl@uni-a.de>  
**Maintainer:** Leonie Schmidt <leonie1.schmidt@uni-a.de>  
**ROS 2 build system:** ament_cmake

## Changes Compared to the Original

### SRDF (`srdf/manipulator_macro.srdf.xacro`)
- Additional planning group **`mia_hand`** (Prensilia Mia Hand) with joints `j_thumb_fle`, `j_thumb_opp`, `j_index_fle`, `j_mrl_fle`
- Predefined hand states: `hand_open`, `hand_close`
- Mia Hand joints declared as **passive joints**
- Additional `disable_collisions` entries for camera mount (`ur_realsense_mount_link`), hand links, and base links

### Kinematics (`config/kinematics.yaml`)
- Solver switched to **TRAC-IK** (`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`)
- Timeout increased to 0.1 s, solution strategy: `Distance`

### Planning (`config/chomp_planning.yaml`)
- **CHOMP** planner added as an additional planning pipeline

### Launch (`launch/ur_moveit_launch.py`)
- Package name updated to `manipulator_ur_moveit_config`
- SRDF path points to local `manipulator.srdf.xacro`
