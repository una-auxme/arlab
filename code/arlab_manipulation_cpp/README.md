# ARLAB Manipulation Cpp

This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

**Maintainer:** Leonie Schmidt &lt;<leonie1.schmidt@uni-a.de>&gt;  
**License:** MIT  
**ROS 2 build system:** ament_cmake

## Overview

`arlab_manipulation_cpp` is a ROS 2 C++ package that controls the manipulator
with an attached robotic hand. It exposes a single ROS 2 action interface
(`/orchestrator/action`) through which the external manipulator orchestrator can trigger
high-level manipulation commands such as picking and placing objects, opening and closing
the hand, or moving the arm to the home position or arbitrary poses.

## Package Structure

```txt
arlab_manipulation_cpp/
├── include/                
│   └── arlab_manipulation_cpp/
│       ├── arm_motion.hpp              # ArmMotion class – MoveIt motion planning & execution
│       ├── hand_motion.hpp             # HandMotion class – Mia Hand grasp action client
│       ├── job_runner.hpp              # JobRunner class – command dispatcher
│       ├── manipulator_exception.hpp   # ManipulationException – domain exception with error codes
│       └── orchestrator_listener.hpp   # OrchestratorActionServer – ROS 2 action server entry point
├── src/
│   ├── arm_motion.cpp                  # Implements pose, Cartesian, box-goal & joint-space motion
│   ├── hand_motion.cpp                 # Implements open, close & raw grasp via action client
│   ├── job_runner.cpp                  # Implements command routing and pick/place sequences
│   ├── manipulator_exception.cpp       # Implements error code to message mapping
│   └── orchestrator_listener.cpp       # Implements action server callbacks & main()
```

## Package Workflow

Internally the package workflow is structured in three layers.
The OrchestatorActionsServer, the Jobrunner and the two Hardware-Level motion wrappers.

```text
    Manipulation Orchestrator (external)
              |
              │  /orchestrator/action
              ▼
    OrchestratorActionServer      ← validates goals, manages threads
              │
              ▼
          JobRunner               ← maps command strings to motion sequences
              │
      ┌───────┴───────┐
      ▼               ▼
  ArmMotion       HandMotion      ← hardware-level motion wrappers
      │               │
    MoveIt         Mia Hand
  (MoveGroup      (grasp action)
  Interface)
```

## Quickstart

### 1. Launch the full manipulation stack (recommended)

The recommended way to start the package is via the central launch file of the
`arlab_manipulation` package, which starts all needed nodes in the correct order:

```bash
ros2 launch arlab_manipulation launch.py
```

> For more information on the full manipulation stack see the README of [`arlab_manipulation`](../arlab_manipulation/README.md).

### 2. Launch this node individually

If you only need the MoveIt interface without the rest of the stack, the node
can also be started in isolation. Note that without the orchestrator and
gripping parameter service running, only raw action goals sent directly to
`/orchestrator/action` will work.

```bash
ros2 run arlab_manipulation_cpp Manipulation_CPP
```
