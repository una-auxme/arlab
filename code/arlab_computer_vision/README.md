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

## Model & Label Configuration

Which models exist and what their detections mean is configured in `config/`,
not in the node's source:

- `config/models.yaml`: the available YOLO models. Each key must match a
  `MODEL_<KEY>` constant in `VisionSnapshotCommand.msg`, which is the ID a
  snapshot request uses to select it. `weights` is a filename relative to
  `yolo_weights/`. `pinned: true` models are loaded at startup and run on every
  frame; the others are loaded on first request and released again after
  `model_ttl_minutes` of disuse.
- `config/labels.yaml`: maps each YOLO class label to a KB entity. `type`
  becomes an `EntityType` constant, `object_category` an
  `EntityPickable.OBJECT_CATEGORY_*` constant (pickables only), and
  `attributes` sets fields on the matching `EntityFurniture` submessage — e.g.
  `dishwasher-open` is stored as a dishwasher whose `open` field is `"open"`.

Labels absent from `labels.yaml` still produce an entity, falling back to a
pickable of unknown category with a runtime warning. An unknown `type`,
`object_category` or model key aborts node startup.

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

- `model_ttl_minutes` (int, default: `10`)
  - Idle time after which a non-pinned model is released from GPU memory
  - Model selection itself is configured in `config/models.yaml`, not via a parameter
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
- `config/models.yaml`: available YOLO models and their loading policy
- `config/labels.yaml`: YOLO label -> KB entity mapping
- `yolo_weights/`: shipped `.pt` weights
- `object_detection.py`: the complete CV -> geometry -> KB pipeline
