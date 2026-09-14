# ARLAB manipulation

This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

**Maintainer:** Sofia Öttl &lt;<sofia.oettl@uni-a.de>&gt;, Christopher Müller &lt;<christopher.mueller@uni-a.de>&gt;, Marc Stumpp &lt;<marc.stumpp@uni-a.de>&gt;  
**License:** MIT  
**ROS 2 build system:** ament_python

## Overview

ROS2 Python package for orchestrating manipulation commands (move, pick with different grasps, place, home, open, close with different grasps)
on the Zirbi robot. It bridges the high-level decision maker and the MoveIt motion planner by
querying the knowledge base for object properties, fetching gripping parameters from a dedicated
service, and forwarding computed poses to the C++ interface node for execution.
After a pick or place the orcestrator asks the force monitor for a dropped object and reports the outcome.

> **Note:** Some functions (octomap utilities, point cloud / bounding box transforms) are still experimental and have not been fully validated on real hardware.

---

## Package structure

```text
arlab_manipulation/           
├── arlab_manipulation/
│    ├── force_monitor.py               # Node: detects load drops on the hand force sensors
│    ├── orchestrator.py                # Main node: receives goals, runs service chain
│    ├── services/
│    │   └── gripping_parameter.py      # Service node: maps objects to grip parameters
│    └── utils/
│        ├── __init__.py
│        ├── octomap_utils.py           # Shelf floor detection, placement search (experimental)
│        └── transform_utils.py         # TF2 helpers for poses and point clouds (experimental)
├── launch/
│   └── launch.py                       # Launches all nodes in the correct order
```

## Package workflow

The orchestrator receives a goal from the decision maker and sequentially queries the knowledge
base and gripping parameter service before forwarding the computed pose to the C++ interface node:

```text
DecisionMaker
    │  ManipulationAction  (/manipulation/action)
    ▼
orchestrator (Python node)
    ├── GetEntity              (/arlab/knowledge/get_entity)   ← knowledge base
    ├── GetShape               (/arlab/knowledge/get_shape)    ← knowledge base
    ├── GetGrippingParameter   (GetGrippingParameter)          ← gripping_parameter node
    ├── OrchestratorAction     (/orchestrator/action)
    │           │
    │           ▼
    │   Manipulation_CPP (C++ node)  →  MoveIt / Robot Controller
    │
    ├── GetObjectDropped       (/object_dropped)               ← force_monitor node
    └── /object_placed         (std_msgs/Bool)                 → decision maker
```

The force monitor itself is armed and disarmed by the C++ node during the pick and place
sequences. The orchestrator only asks for the resulting status after the motion has finished
and maps a detected drop to the `OBJECT_DROPPED` error code in its action result.

---

## Quickstart

### 1. Launch the full manipulation stack

```bash
ros2 launch arlab_manipulation launch.py
```

This starts the following nodes in order:

1. **`gripping_parameter`** — service node, started first so it is ready when the orchestrator receives its first goal
2. **`force_monitor`** — watches the hand force sensors for load drops
3. **`orchestrator`** — Python orchestrator node
4. **`Manipulation_CPP`** — C++ MoveIt interface node

### 2. Send a manipulation goal (example)

```bash
ros2 action send_goal /manipulation/action arlab_common_interfaces/action/ManipulationAction \
  "{command: {command_type: 'pick', target_entityid: 'apple_01'}}"
```
