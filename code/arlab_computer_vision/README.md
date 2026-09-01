# arlab_computer_vision (ROS 2)

## Maintainers

- Simeon Wagner <simeon.wagner@uni-a.de>
- Lars Britz    <lars.britz@uni-a.de>

YOLO-based object detection/segmentation integrated with the project’s knowledge base (KB).

## Node: `ObjectDetection`

Implementation: `arlab_computer_vision/object_detection.py`

High-level dataflow:

1. Receive `camera_color_image` (+ `camera_info` and optionally `camera_point_cloud`)
2. Run every pinned Ultralytics YOLO model on the frame, plus any extra models the
   snapshot request asked for
3. Convert detections to 3D geometry using TF + (when enabled) point cloud + camera intrinsics
4. Map each YOLO label to a KB entity type via `config/labels.yaml`
5. Create/update KB entities via ROS services

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
  `attributes` sets fields on the matching `EntityFurniture` submessage (e.g.
  `dishwasher-open` is stored as a dishwasher whose `open` field is `"open"`).

Labels absent from `labels.yaml` still produce an entity, falling back to a
pickable of unknown category with a runtime warning. An unknown `type`,
`object_category` or model key aborts node startup.

## Expected Inputs

The node subscribes to these topics:

- `camera_color_image`: `sensor_msgs/Image`
- `camera_info`: `sensor_msgs/CameraInfo`
- `camera_point_cloud`: `sensor_msgs/PointCloud2` (required when `use_depth=true`)

Concrete namespaces are provided via ROS topic remappings in the launch files.

## Action Interface: `/vision/snapshot`

With `snapshot_mode=true` (the default, and what the launch file sets), the node
does not process frames on its own but only acts when it receives a goal on the
`/vision/snapshot` action, of type `arlab_common_interfaces/VisionSnapshotAction`.
This is the normal way to drive the node.

On a goal the node discards its cached frame and waits up to 5 s for a fresh
RGB + point cloud pair, so the snapshot reflects the robot's settled pose rather
than a buffered frame from before it moved.

The goal carries a `VisionSnapshotCommand`:

- `clear_database` (bool): delete the previously stored pickables after this
  snapshot's detections have been added
- `mask_hand` (bool): blank the lower 20% of each mask, suppressing detections of
  the robot's own gripper
- `extra_models` (uint8[]): additional models to run for this snapshot, on top of
  the ones `models.yaml` marks as pinned. Use the `MODEL_*` constants from
  `VisionSnapshotCommand.msg`.

The result carries a `VisionSnapshotResponse` with `result` set to one of:

- `SUCCESS` (0)
- `ERROR_UNKNOWN` (1): processing raised; details in `error_msg`
- `ERROR_NO_IMAGE_DATA` (2): no camera frame arrived within the wait window

The goal is always *succeeded*, even on failure. Errors are reported through
the `result` carried by `VisionSnapshotResponse`, so clients must inspect the response code.

There is no feedback topic. `test/test_snapshot_action.py` is a manual client
that sends one goal and saves the annotated frame.

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
- `target_frame` (string, node default: `camera_tool_link`)
  - TF target frame for pose and point cloud geometry
  - For semantic annotation purposes in movement **the launch file overrides this to `world`**
- `sync_tolerance` (double, node default: `0.5`)
  - Approximate time sync tolerance in seconds between RGB and point cloud
  - **The launch file overrides this to `5.0`**
- `visualize` (bool, default: `true`)
  - Publish the annotated frame on `/vision/segmented_image`
- `max_image_width` (int, default: `640`)
  - Frames are scaled to this square size before inference; `0` keeps the original
    resolution. Lower values are faster and use less memory, at the cost of
    detection quality on small objects.

Remaining parameters, rarely changed: `log_level` (string, `INFO`),
`use_clustering` (bool, `true` keeps only the largest DBSCAN cluster per
detection), and
`clear_db_on_no_detection` (bool, `true`: clears the KB when within a frame
nothing was detected ).

## Launch / Quick Start

Integrated launch:

```bash
ros2 launch arlab_computer_vision object_detection_launch.py
```

Notes:

- The launch file configures `snapshot_mode: True` (action-driven processing), so
  nothing happens until a `/vision/snapshot` goal arrives.
- It remaps the logical camera topics under the `/camera_gripper/...` namespace.
- It overrides `sync_tolerance` to `5.0` and sets `target_frame` from a launch
  argument defaulting to `world`(This was done by movement for semantic annotation reasons):

```bash
ros2 launch arlab_computer_vision object_detection_launch.py target_frame:=camera_tool_link
```

## Where to Look (fast navigation)

- `launch/object_detection_launch.py`: topic remappings + default parameter set
- `config/models.yaml`: available YOLO models and their loading policy
- `config/labels.yaml`: YOLO label -> KB entity mapping
- `yolo_weights/`: shipped `.pt` weights, currently `person.pt` and `open_img.pt`.
  Note that `models.yaml` also references `ycb.pt`, which is **not** shipped; that
  model logs a warning and yields no detections until the file is added.
- `test/test_snapshot_action.py`: manual client for the `/vision/snapshot` action
- `object_detection.py`: the complete CV -> geometry -> KB pipeline
