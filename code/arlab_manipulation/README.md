# ARLAB manipulation

ROS2 Python package for orchestrating commands from desicion maker for manipulator processing with use of knowledgebase and gripping parameter service.
This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

> **Note:** Some functions (octomap utilities, point cloud / bounding box transforms) are still experimental and have not been fully validated on real hardware.

---

## Architecture

```DecisionMaker
    │  ManipulationAction  (/manipulation/action)
    ▼
orchestrator (Python node)
    ├── GetEntity          (/arlab/knowledge/get_entity)   ← knowledge base
    ├── GetShape           (/arlab/knowledge/get_shape)    ← knowledge base
    ├── GetGrippingParameter (GetGrippingParameter)        ← gripping_parameter node
    └── OrchestratorAction (/orchestrator/action)
            │
            ▼
    Manipulation_CPP (C++ node)  →  MoveIt / Robot Controller```

## Quickstart

### 1. Launch the full manipulation stack:

```bash
ros2 launch arlab_manipulation launch.py
```

This starts:
1. **`gripping_parameter`** — parameter service (ready before orchestrator)
2. **`orchestrator`** — Python orchestrator node
3. **`Manipulation_CPP`** — C++ MoveIt interface node

### 2. Send a manipulation goal (example for testing)

```bash
ros2 action send_goal /manipulation/action arlab_common_interfaces/action/ManipulationAction \
  "{command: {command_type: 'pick', target_entityid: 'apple_01'}}"```
