# Test Scripts for predict_snapshot

## Overview

Test scripts for the snapshot-based prediction mode and ROS2 Action Server.

## Available Test Scripts

### 1. `test_predict_snapshot_simple.py`
Tests the `predict_snapshot()` function directly (without Action Server).

```bash
# Terminal 1: Start object detection node
ros2 launch arlab_computer_vision object_detection_launch.py

# Terminal 2: Run test script
python3 scripts/test_predict_snapshot_simple.py
```

### 2. `test_predict_snapshot_action_client.py`
Tests the ROS2 Action Server (`predict_snapshot` action).

```bash
# Terminal 1: Start object detection node
ros2 launch arlab_computer_vision object_detection_launch.py

# Terminal 2: Run action client test
python3 scripts/test_predict_snapshot_action_client.py
```

## Prerequisites

1. Camera must be running and publishing:
   - `/camera/color/image_raw` (RGB images)
   - `/camera/color/camera_info` (camera intrinsics)
   - `/camera/depth/image_rect_raw` (optional, depth images)

2. Knowledge base services must be available (if entities should be saved)

## Expected Behavior

1. Node starts and loads YOLO model
2. Node subscribes to `camera_info` and receives intrinsics
3. Test script captures RGB (+ optional Depth) image from topics
4. `predict_snapshot` is called (directly or via Action)
5. YOLO inference runs (should be fast, ~4-8ms after warmup)
6. Entities are detected and returned
7. Entities are also saved to knowledge base (if KB services available)

## Troubleshooting

- **"Camera intrinsics not set"**: Make sure camera is publishing `camera_info` topic
- **"Timeout waiting for RGB image"**: Make sure camera is publishing `/camera/color/image_raw`
- **No entities detected**: Point camera at objects that YOLO can detect (COCO classes)
- **Slow inference**: First inference may be slow due to CUDA JIT compilation
- **Action server not found**: Make sure `arlab_common_interfaces` is built
