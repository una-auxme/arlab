# Computer Vision Launch Files (ROS 2)

## Maintainers

- Aleksander Michalak <aleksander1.michalak@uni-a.de>
- Meruna Yugarajah <m.yugarajah@gmail.com>

This folder contains launch files for the `arlab_computer_vision` package.

## `object_detection_launch.py` (system / integrated run)

This launch file starts the `ObjectDetection` node configured for the integrated workflow (Behavior Tree / decision-making triggers).

What it configures:

- `snapshot_mode: True`
  - The node processes frames only after an action request to `/vision/snapshot`.
- Topic remappings (logical node inputs):
  - `camera_color_image` -> `/camera_gripper/color/image_raw`
  - `camera_info` -> `/camera_gripper/color/camera_info`
  - `camera_point_cloud` -> `/camera_gripper/depth/color/points`
- TF: `target_frame: world`

Typical usage:

```bash
ros2 launch arlab_computer_vision object_detection_launch.py
```
