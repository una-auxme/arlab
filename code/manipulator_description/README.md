# Manipulator Description

This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

**Maintainer:** Leonie Schmidt &lt;<leonie1.schmidt@uni-a.de>&gt;  
**License:** Apache2.0  
**ROS 2 build system:** ament_cmake

## Overview

`manipulator_description` is the central description and launch package for the ARLab manipulation setup. It contains the URDF/xacro robot model, all launch files for the manipulator, controller configuration, and mesh assets.

The robot consists of:

- **UR5e** — 6-DOF industrial arm by Universal Robots. The arm is placed 7 cm above the world origin on top of the pedestal.
- **Mia Hand** — 4-DOF underactuated prosthetic hand by Prensilia. The Hand is attached to `ur_realsense_mount_link`, which is fixed to `tool0` of the UR5e.
- **RealSense D435** — a depth camera mounted on the tool0 via a custom 3D-printed mount
- **Scene objects** — shelf (`SchrankRegal`) at three heights and a robot base pedestal (`ur_sockel`)

The full scene is is assempled in the top-level file `urdf/manipulator.urdf.xacro`.

## Package Structure

```txt
manipulator_description/
├── config/                                   # ROS 2 parameter files
│   ├── manipulator_controllers.yaml          # Controller manager configuration for arm and hand
│   └── ur_joint_limits.yaml                  # Custom UR5e joint position and velocity limits
├── launch/                                   # ROS 2 launch files
│   ├── manipulator.control.robot.launch.py   # Real robot: arm driver + hand driver
│   ├── manipulator.control.sim.launch.py     # Simulation: Gazebo + controllers
│   ├── manipulator.full.robot.launch.py      # Real robot: drivers + MoveIt (recommended)
│   ├── manipulator.full.sim.launch.py        # Simulation: Gazebo + MoveIt (recommended)
│   └── manipulator.moveit.robot.launch.py    # Real robot: MoveIt only
├── meshes/                                   # Visual and collision mesh files
│   ├── halterung_realsense.stl               # RealSense D435 mount bracket
│   ├── SchrankRegal.stl                      # Shelf / rack scene object
│   └── ur_sockel.stl                         # Robot base pedestal
└── urdf/                                     # Robot description files (xacro)
    ├── manipulator.urdf.xacro                # Top-level robot description (entry point)
    ├── mia_hand.ros2_control.xacro           # ros2_control hardware interface for the Mia Hand
    ├── ur_realsense_mount.xacro              # RealSense D435 mount bracket link and joint
    ├── ur_sockel.xacro                       # Robot base pedestal link and joint
    └── schrankregal.xacro                    # Parametric shelf scene object macro
```

## Launch Files

The launch files are structured in two layers — *control* (drivers / simulation) and *full* (control + MoveIt). Always prefer the **full** launch files unless you have a specific reason to start the layers separately.

### Launch file overview

| File | Target | What it starts |
| --- | --- | --- |
| `manipulator.full.robot.launch.py` | **Real robot** ✅ recommended | UR driver + Mia Hand driver + MoveIt + RViz |
| `manipulator.full.sim.launch.py` | **Simulation** ✅ recommended | Gazebo + controllers + MoveIt + RViz |
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
└── manipulator_ur_moveit_config/ur_moveit.launch.py
```

### Common launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `ur_type` | `ur5e` | UR robot model series |
| `robot_ip` | `10.135.245.20` | IP address of the real UR controller |
| `serial_port_arg` | `/dev/ttyUSB0` | Serial port for the Mia Hand |
| `launch_rviz_moveit` | `true` | Open RViz with MoveIt plugin |
| `use_mock_hardware` | `false` (robot) / `true` (sim) | Use mock hardware instead of real drivers |

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
