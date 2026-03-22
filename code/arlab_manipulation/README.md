# ARLAB manipulation

ROS2 Python package for orchestrating manipulation commands from the decision maker, using the knowledge base and gripping parameter service.
This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

> **Note:** Some functions (octomap utilities, point cloud / bounding box transforms) are still experimental and have not been fully validated on real hardware.

---

## Package structure

```text
arlab_manipulation/           
├── arlab_manipulation/
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

```text
DecisionMaker
    │  ManipulationAction  (/manipulation/action)
    ▼
orchestrator (Python node)
    ├── GetEntity              (/arlab/knowledge/get_entity)   ← knowledge base
    ├── GetShape               (/arlab/knowledge/get_shape)    ← knowledge base
    ├── GetGrippingParameter   (GetGrippingParameter)          ← gripping_parameter node
    └── OrchestratorAction     (/orchestrator/action)
                │
                ▼
        Manipulation_CPP (C++ node)  →  MoveIt / Robot Controller
```

---

## Quickstart

### 1. Launch the full manipulation stack

```bash
ros2 launch arlab_manipulation launch.py
```

This starts the following nodes in order:

1. **`gripping_parameter`** — service node, started first so it is ready when the orchestrator receives its first goal
2. **`orchestrator`** — Python orchestrator node
3. **`Manipulation_CPP`** — C++ MoveIt interface node

### 2. Send a manipulation goal (example)

```bash
ros2 action send_goal /manipulation/action arlab_common_interfaces/action/ManipulationAction \
  "{command: {command_type: 'pick', target_entityid: 'apple_01'}}"
```
