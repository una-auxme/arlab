# arlab_perception (ROS 2)

Lean perception helpers with an intentionally small scope right now.

## Maintainers

- Meruna Yugarajah <m.yugarajah@gmail.com>

## Current responsibility (camera)

At the moment, `arlab_perception` does **not** perform any data preprocessing for the vision pipeline.
Instead, `arlab_computer_vision` accesses the camera topics directly (configured via its launch files).

As a result:

- No image/pointcloud filtering, reformatting, or synchronization is done in `arlab_perception` today.
- The perception layer is tightly coupled to the current CV integration.

## Lidar integration status (placeholder)

The Lidar node in this package is a **placeholder** for future LiDAR integration.
It currently provides a simple relay:

- Input topic: `/scan` (`sensor_msgs/LaserScan`)
- Output topic: `/relay_scan` (`sensor_msgs/LaserScan`)

Right now, no other component should depend on `/relay_scan` yet.

## What exists in this repository right now

- `arlab_perception/lidar_data.py`

## Historical nodes (now removed)

Earlier versions of this package contained:

- a `video_node` to replay prerecorded footage for software-only computer vision tests
- a `camera_node` to interface with a temporary camera workaround (e.g. Kinect)

With the current repository setup, those nodes are no longer required:

- the CV pipeline now accesses camera topics directly (`arlab_computer_vision`)
- the previous `video_node` used for training/pipeline testing is no longer needed

## Note on `setup.py`

`code/arlab_perception/setup.py` console-scripts may not be aligned with the currently present modules.
For the current repo state, focus on the implementation files themselves (e.g. `arlab_perception/lidar_data.py`) and align `entry_points` if you want to start nodes via `ros2 run` immediately.
