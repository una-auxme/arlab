# arlab_computer_vision (ROS 2)

## Maintainers

- Aleksander Michalak <aleksander1.michalak@uni-a.de>
- Meruna Yugarajah <m.yugarajah@gmail.com>

YOLO-based object detection/segmentation integrated with the project’s knowledge base (KB).

## Node: `ObjectDetection`

Implementation: `arlab_computer_vision/object_detection.py`

High-level dataflow:

1. Receive `camera_color_image` (+ `camera_info` and optionally `camera_point_cloud`)
2. Run Ultralytics YOLO on the incoming frame
3. Convert detections to 3D geometry using TF + (when enabled) point cloud + camera intrinsics
4. Create/update KB entities via ROS services

## Expected Inputs

The node subscribes to these *logical* topics:

- `camera_color_image`: `sensor_msgs/Image`
- `camera_info`: `sensor_msgs/CameraInfo`
- `camera_point_cloud`: `sensor_msgs/PointCloud2` (required when `use_depth=true`)

Concrete namespaces are provided via ROS topic remappings in the launch files.

## Knowledge Base Integration

Service prefix is hard-coded in the node as `/arlab/knowledge`.

Used services:

- `/arlab/knowledge/get_entities`
- `/arlab/knowledge/add_entity`
- `/arlab/knowledge/del_entities`
- `/arlab/knowledge/upd_shape`

## Outputs (debug / visualization)

Depending on parameters, the node can publish:

- `/vision/segmented_image` (when `visualize=true`)
- `/vision/debug_pc` (entity point cloud used for geometry)

## Important Parameters (ROS params)

- `yolo_weights` (string)
  - Path to the `.pt` file used for YOLO inference (default is `yolo_weights/yolo11n-seg_demo_day.pt`)
- `use_depth` (bool, default: `true`)
  - If `true`, the node needs synchronized `camera_info` + `camera_point_cloud` to build 3D geometry
- `snapshot_mode` (bool, default: `true`)
  - If `true`, processing happens only when `/vision/snapshot` is requested (action-driven)
  - If `false`, the node processes frames continuously
- `target_frame` (string)
  - TF target frame for pose and point cloud geometry (e.g. `world`, `camera_link`, ...)
- `sync_tolerance` (double)
  - Approximate time sync tolerance between RGB and point cloud

## Launch / Quick Start

Integrated launch:

```bash
ros2 launch arlab_computer_vision object_detection_launch.py
```

Notes:

- The launch file configures `snapshot_mode: True` (action-driven processing).
- It remaps the logical camera topics under the `/camera_gripper/...` namespace.

## Where to Look (fast navigation)

- `launch/object_detection_launch.py`: topic remappings + default parameter set
- `yolo_weights/`: shipped `.pt` weights
- `object_detection.py`: the complete CV -> geometry -> KB pipeline
