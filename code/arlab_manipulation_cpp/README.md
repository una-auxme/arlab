# arlab_manipulation_cpp

**Maintainer:** Leonie Schmidt &lt;<leonie1.schmidt@uni-a.de>&gt;  
**License:** MIT  
**ROS 2 build system:** ament_cmake  

## Overview

`arlab_manipulation_cpp` is a ROS 2 C++ package that controls a **UR manipulator arm**
with an attached **Mia robotic hand**. It exposes a single ROS 2 action interface
(`/orchestrator/action`) through which the external manipulator orchestrator can trigger
high-level manipulation commands such as picking and placing objects, opening and closing
the hand, or moving the arm to the home position or arbitrary poses.

## Package Structure

```txt
arlab_manipulation_cpp/
├── include/                            # Public header files
│   └── arlab_manipulation_cpp/         # C++ Package-namespaced header directory
│       ├── arm_motion.hpp
│       ├── hand_motion.hpp
│       ├── job_runner.hpp
│       ├── manipulator_exception.hpp
│       └── orchestrator_listener.hpp
├── src/                                # Implementation files
│   ├── arm_motion.cpp
│   ├── hand_motion.cpp
│   ├── job_runner.cpp
│   ├── manipulator_exception.cpp
│   └── orchestrator_listener.cpp
```

## File Descriptions

| File | Description |
| --- | --- |
| `orchestrator_listener.hpp/.cpp` | ROS 2 action server; entry point of the package. Owns all motion components and initilizes the OrchestratorActionServer. |
| `job_runner.hpp/.cpp` | Command dispatcher. Reads the command string from the action message and calls the appropriate arm/hand motion methods. |
| `arm_motion.hpp/.cpp` | Wrapper around `MoveGroupInterface`. Provides pose goals, Cartesian paths, box-constrained goals, and joint-space goals. |
| `hand_motion.hpp/.cpp` | Wrapper around the Mia Hand grasp action client. Provides `Open()`, `Close()`, and a low-level `Grasp()`. |
| `manipulator_exception.hpp/.cpp` | Domain exception class that carries a numeric error code and a human-readable message. |

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
