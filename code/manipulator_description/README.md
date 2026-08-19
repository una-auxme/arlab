# Manipulator Description

This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

**Maintainer:** Leonie Schmidt &lt;<leonie1.schmidt@uni-a.de>&gt;, Luka Eberle &lt;<luka.eberle@uni-a.de>&gt;, Julian Schropp &lt;<julian-schropp@web.de>&gt;, Michael Wallner &lt;<michael.wallner@uni-a.de>&gt; 
**License:** Apache2.0  
**ROS 2 build system:** ament_cmake

## Overview

`manipulator_description` is the central description and launch package for the ARLab manipulation setup. It contains the URDF/xacro robot model, all launch files for the manipulator, controller configuration, and mesh assets.

The robot consists of:

- **UR5e** — 6-DOF industrial arm by Universal Robots. The arm is placed 7 cm above the world origin on top of the pedestal.
- **Mia Hand** — 4-DOF underactuated prosthetic hand by Prensilia. The Hand is attached to `ur_realsense_mount_link`, which is fixed to `tool0` of the UR5e.
- **Ranger Air** — Omnidirectional Robot base
- **Lidar Sensor** — Lidar Sensor for Nav2
- **Chassis** — Placeholder chassis between Robot base and Manipulator
- **RealSense D435** — a depth camera mounted on the tool0 via a custom 3D-printed mount
- **Scene objects** — truncated world file from AWS robotics (seperate Package: small_house) &lt;<https://github.com/aws-robotics/aws-robomaker-small-house-world>&gt;

The full scene is is assembled in the top-level file `urdf/manipulator_mobile.urdf.xacro`.

## Package Structure

```txt
manipulator_description/
├── config/                                   # ROS 2 parameter files
│   ├── joint_names_agx_description.yaml      # Joints for Base
│   ├── manipulator_controllers.yaml          # Controller manager configuration for arm and hand
│   └── ur_joint_limits.yaml                  # Custom UR5e joint position and velocity limits
├── launch/                                   # ROS 2 launch files
│   ├── manipulator.control.robot.launch.py   # Real robot: arm driver + hand driver
│   ├── manipulator.control.sim.launch.py     # Simulation: Gazebo + controllers
│   ├── manipulator.full.robot.launch.py      # Real robot: drivers + MoveIt (recommended)
│   ├── manipulator.full.sim.launch.py        # Simulation: Gazebo + MoveIt (recommended)
│   └── manipulator.moveit.robot.launch.py    # Real robot: MoveIt only
├── meshes/                                   # Visual and collision mesh files
│   ├── dae/                                  # Various meshes for Robot Base
│   ├── chasis.mtl                            # Material for chasis.obj
│   ├── chasis.obj                            # 3d file for chasis
│   ├── d435.mtl                              # Material for d435.obj
│   ├── d435.obj                              # 3d file for d435 (intel RealSense camera)
│   ├── halterung_realsense.stl               # RealSense D435 mount bracket
│   ├── SchrankRegal.stl                      # Shelf / rack scene object
│   └── sockel_robot.stl                      # Robot base pedestal
└── urdf/                                     # Robot description files (xacro)
    ├── _d435.urdf.xacro                      # camera xacro file
    ├── manipulator_mobile.urdf.xacro         # Top-level robot description (entry point)
    ├── mia_hand_left.urdf.xacro              # xacro file for modified mia hand (left) (modified because of filepath problems with gazebo)
    ├── mia_hand_right.urdf.xacro             # xacro file for modified mia hand (right) (modified because of filepath problems with gazebo)
    ├── mia_hand.ros2_control.xacro           # ros2_control hardware interface for the Mia Hand
    ├── mia_hand.urdf.xacro                   # base xacro for mia hand (chooses left or right based on parameter)
    ├── ranger_air_description.csv            # list of physic parameters
    ├── ranger_air_description.xacro          # xacro for robot base (ranger air model)
    ├── schrankregal.xacro                    # Parametric shelf scene object macro
    ├── ur_chasis.xacro                       # placeholder xacro for chasis 
    └── ur_realsense_mount.xacro              # RealSense D435 mount bracket link and joint
```

## Launch Files

The launch files are structured in two layers — *control* (drivers / simulation) and *full* (control + MoveIt). Always prefer the **full** launch files unless you have a specific reason to start the layers separately.

### Launch file overview

| File | Target | What it starts |
| --- | --- | --- |
| `manipulator.full.robot.launch.py` | **Real robot** ✅ recommended | UR driver + Mia Hand driver + MoveIt + RViz |
| `manipulator.full.sim.launch.py` | **Simulation** ✅ recommended | Gazebo + controllers + RViz(MoveIt) + RViz(Nav2) |
| `manipulator.control.robot.launch.py` | Real robot | UR driver + Mia Hand driver only |
| `manipulator.control.sim.launch.py` | Simulation | Gazebo + all controllers only |
| `manipulator.moveit.robot.launch.py` | Real robot | MoveIt + RViz only (drivers must already be running) |

### Launch file dependency graph

```txt
manipulator.full.robot.launch.py
├── manipulator.control.robot.launch.py
│   ├── manipulator_robot_driver/ur_control.launch.py
│   └── mia_hand_driver/mia_hand_driver_launch.py
└── manipulator.moveit.robot.launch.py
    └── manipulator_ur_moveit_config/ur_moveit.launch.py

manipulator.full.sim.launch.py
├── manipulator.control.sim.launch.py
│   ├── Gazebo (ros_gz_sim)
│   ├── robot_state_publisher
│   └── controller_manager spawners
├── manipulator_ur_moveit_config/ur_moveit.launch.py
└── arlab_movement/launch
    ├── ranger_nav2.launch.py
    └── slam_async.launch.py
```

### Common launch arguments for manipulator.full.robot.launch.py

| Argument | Default | Description |
| --- | --- | --- |
| `ur_type` | `ur5e` | UR robot model series |
| `robot_ip` | `10.135.245.20` | IP address of the real UR controller |
| `serial_port_arg` | `/dev/ttyUSB0` | Serial port for the Mia Hand |
| `launch_rviz_moveit` | `true` | Open RViz with MoveIt plugin |
| `use_mock_hardware` | `false` (robot) / `true` (sim) | Use mock hardware instead of real drivers |


### Common launch arguments for manipulator.full.sim.launch.py


| Argument | Default | Description |
| --- | --- | --- |
| `ur_type` | `ur5e` | UR robot model series |
| `safety_limits` | `true` | wether to use safety limits for ur |
| `spawn_x` | `-4` | spawn coordinate X of Robot |
| `spawn_y` | `1` | spawn coordinate Y of Robot |
| `spawn_z` | `0` | spawn coordinate Z of Robot |
| `spawn_roll` | `0` | spawn roll of Robot |
| `spawn_pitch` | `0` | spawn pitch of Robot |
| `spawn_yaw` | `0` | spawn yaw of Robot |
| `world_file` | `PathJoinSubstitution([house_pkg_share, "worlds", "small_house.world"])`<-- whatever this generates | world file for robot |
| `gazebo_gui` | `true` | wether to launch gazebo simulation |
| `launch_rviz` | `true` | RViz Window for Moveit and Robot view |
| `launch_slam` | `true` | wether to launch slam |
| `launch_nav2` | `true` | wether to launch nav2 |
| `launch_nav2_rviz` | `true` | RViz Window for Nav2 control |




## Quickstart

### Real robot (recommended)

```bash
ros2 launch manipulator_description manipulator.full.robot.launch.py \
  robot_ip:=10.135.245.20 \
  serial_port_arg:=/dev/ttyUSB0
```

### Simulation (recommended)

```bash
ros2 launch manipulator_description manipulator.full.sim.launch.py
```

### Control only (no MoveIt)

```bash
# Real robot
ros2 launch manipulator_description manipulator.control.robot.launch.py

# Simulation
ros2 launch manipulator_description manipulator.control.sim.launch.py
```

> For more information on the full manipulation stack see the README of [`arlab_manipulation`](../arlab_manipulation/README.md).
