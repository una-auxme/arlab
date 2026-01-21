# ARLab Computer Vision

ROS2 node for snapshot-based object detection using YOLO segmentation.

## Features

- **Snapshot-based prediction**: On-demand object detection via ROS2 Action
- **YOLO segmentation**: Detects and segments objects in RGB images
- **Depth integration**: Optional depth image processing for 3D pose estimation
- **Knowledge Base integration**: Saves detected entities to ROS2 knowledge base
- **GPU acceleration**: CUDA support for fast inference

## Quick Start

```bash
# 1. Install dependencies (see INSTALL.md)
# 2. Build workspace
colcon build --packages-select arlab_computer_vision
source install/setup.bash

# 3. Start node
ros2 launch arlab_computer_vision object_detection_launch.py

# 4. Use action server
# See scripts/test_predict_snapshot_action_client.py for example
```

## Usage

### ROS2 Action Server

The node provides a `predict_snapshot` action:

```python
from arlab_common_interfaces.action import PredictSnapshot
from rclpy.action import ActionClient

# Send goal with RGB and optional depth image
goal = PredictSnapshot.Goal()
goal.rgb_image = rgb_image_msg
goal.depth_image = depth_image_msg  # optional

# Receive feedback and result
# See scripts/test_predict_snapshot_action_client.py for full example
```

### Direct Function Call

```python
from arlab_computer_vision.object_detection import ObjectDetection

node = ObjectDetection()
entities = await node.predict_snapshot(rgb_image, depth_image)
```

## Files

- `INSTALL.md` - Installation instructions
- `requirements*.txt` - Python dependencies (CUDA/CPU/ROCm)
- `scripts/` - Test scripts and examples
- `launch/` - ROS2 launch files

## Documentation

See `INSTALL.md` for detailed installation and setup instructions.
