# arlab_manipulation

ROS2 Python package for orchestrating commands from desicion maker for manipulator processing with use of knowledgebase and gripping parameter service.
This project was built specifically for the `ARLab` at the University of Augsburg and developed in the context of the `Zirbi` robot.

> **Note:** Some functions (octomap utilities, point cloud / bounding box transforms) are still experimental and have not been fully validated on real hardware.

---

## Architecture

```
DecisionMaker
    │  ManipulationAction  (/manipulation/action)
    ▼
orchestrator (Python node)
    ├── GetEntity          (/arlab/knowledge/get_entity)   ← knowledge base
    ├── GetShape           (/arlab/knowledge/get_shape)    ← knowledge base
    ├── GetGrippingParameter (GetGrippingParameter)        ← gripping_parameter node
    └── OrchestratorAction (/orchestrator/action)
            │
            ▼
    Manipulation_CPP (C++ node)  →  MoveIt / Robot Controller
```

---

## Package contents

| File | Description |
|------|-------------|
| `orchestrator.py` | Main ROS2 node. Receives goals, runs the service chain, and dispatches poses. |
| `gripping_parameter.py` | Service node mapping object name / shape group → grip force, modes, weight. |
| `utils/octomap_utils.py` | Shelf-floor detection, box collision check, free-placement search. *(experimental)* |
| `utils/transform_utils.py` | TF2 helpers for poses, point clouds, and bounding boxes. *(experimental)* |
| `launch/launch.py` | Launches all three nodes in the correct startup order. |

---

## Dependencies

**ROS2 packages**

- `rclpy`
- `tf2_ros`, `tf2_geometry_msgs`
- `moveit_msgs`
- `sensor_msgs`, `sensor_msgs_py`
- `geometry_msgs`, `std_msgs`
- `arlab_common_interfaces` — custom action/service/message definitions
- `arlab_knowledge_interfaces` — knowledge base service definitions
- `arlab_manipulation_cpp` — C++ subscriber / MoveIt interface node

**Python**

- `numpy`

---

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
  "{command: {command_type: 'pick', target_entityid: 'apple_01'}}"
```

---

## Error codes

| Code | Meaning |
|------|---------|
| `1` | Success |
| `-40` | Octomap is empty |
| `-41` | `GetEntity` service call failed |
| `-42` | `GetShape` service call failed |
| `-43` | `GetGrippingParameter` service call failed |
| `-44` | `OrchestratorAction` server not available |
| `-45` | `OrchestratorAction` goal rejected |
| `-46` | Failed to receive orchestrator goal handle |
| `-48` | Failed to process orchestrator action result |
| `-49` | No free placing area found in octomap |
| `-50` | Bounding box is empty |
| `-51` | TF2 transform failed |
| `-52` | No pose available for pick |

---
