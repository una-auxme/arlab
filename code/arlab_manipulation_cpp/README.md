# ARLAB Manipulation Cpp

This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

**Maintainer:** Leonie Schmidt &lt;<leonie1.schmidt@uni-a.de>&gt;, Christopher Müller &lt;<christopher.mueller@uni-a.de>&gt;, Marc Stumpp &lt;<marc.stumpp@uni-a.de>&gt;  
**License:** MIT  
**ROS 2 build system:** ament_cmake

## Overview

`arlab_manipulation_cpp` is a ROS 2 C++ package that controls the manipulator
with an attached robotic hand. It exposes a single ROS 2 action interface
(`/orchestrator/action`) through which the external manipulator orchestrator can trigger
high-level manipulation commands such as picking and placing objects, opening and closing
the hand in multiple different ways, or moving the arm to the home position or arbitrary poses.
During a pick it also enables the hand force data stream and arms the external force monitor
node, so that an object lost during transport can be detected through the hand force sensors.

## Package Structure

```txt
arlab_manipulation_cpp/
├── include/
│   └── arlab_manipulation_cpp/
│       ├── arm_motion.hpp              # ArmMotion class – MoveIt motion planning & execution
│       ├── force_monitor_switch.hpp    # ForceMonitorSwitch – arms/disarms the force monitor node
│       ├── hand_force_switch.hpp       # HandForceSwitch – switches the Mia Hand force data stream
│       ├── hand_motion.hpp             # HandMotion class – Mia Hand grasp action client
│       ├── job_runner.hpp              # JobRunner class – command dispatcher
│       ├── manipulator_exception.hpp   # ManipulationException – domain exception with error codes
│       └── orchestrator_listener.hpp   # OrchestratorActionServer – ROS 2 action server entry point
└── src/
    ├── arm_motion.cpp                  # Implements pose, Cartesian, box-goal & joint-space motion
    ├── force_monitor_switch.cpp        # Implements the ActivateForceMonitor service client
    ├── hand_force_switch.cpp           # Implements the SetBool stream switch client
    ├── hand_motion.cpp                 # Implements open, close & raw grasp via action client
    ├── job_runner.cpp                  # Implements command routing and pick/place sequences
    ├── manipulator_exception.cpp       # Implements error code to message mapping
    └── orchestrator_listener.cpp       # Implements action server callbacks & main()
```

## Package Workflow

An incoming action goal is received by the `OrchestratorActionServer`, handed to the `JobRunner`
which resolves the command into a concrete motion sequence, that is then executed by `ArmMotion`
via MoveIt or `HandMotion` via the Mia Hand grasp action. For pick and place sequences the
`JobRunner` additionally switches the Mia Hand force data stream via `HandForceSwitch` and arms
the external force monitor via `ForceMonitorSwitch`.

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
      ┌───────┴────────┬──────────────┬──────────────────┐
      ▼                ▼              ▼                  ▼
  ArmMotion       HandMotion   HandForceSwitch  ForceMonitorSwitch
      │                │              │                  │
    MoveIt         Mia Hand       Mia Hand         force_monitor
  (MoveGroup    (grasp action) (stream switch)  (activation service)
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
`/orchestrator/action` will work. The force monitor node is optional: if it is
not running, the switches log a warning and the motion sequence continues
without drop detection.

```bash
ros2 run arlab_manipulation_cpp Manipulation_CPP
```
