# ARLAB bringup

This package provides launch configurations for the distributed robot system.

## Package structure

```txt
arlab_bringup/
├── arlab_bringup/
│   ├── __init__.py              # Package initialization
│   └── dummy.py                 # Dummy node for testing
├── launch/
│   ├── arlab-nuc.launch.py      # NUC hardware launch file
│   ├── jetson-arlab1.launch.py  # Jetson ARLAB1 launch file
│   ├── jetson-arlab2.launch.py  # Jetson ARLAB2 launch file
│   └── jetson-arlab3.launch.py  # Jetson ARLAB3 launch file
├── setup.py                     # Package configuration
└── package.xml                 # ROS2 package manifest
```

## Startup procedure

Each compute node of the distributed system has their own launch file.

1. **Start ARLAB1 on Jetson arlab1**:

   ```bash
   ros2 launch arlab_bringup jetson-arlab1.launch.py
   ```

   This launches robot control and gripper camera calibration on ARLAB1. The ARLAB1 is connected to the MIA hand via USB.

2. **Start NUC on arlab-nuc**:

   ```bash
   ros2 launch arlab_bringup arlab-nuc.launch.py
   ```

   This launches MoveIt robot planning and manipulation nodes on the NUC. The NUC is also connected to the gripper_camera (camera driver runs here due to issues on ARM/Jetson).

3. **Start ARLAB2/3 (any order)**:

   ```bash
   ros2 launch arlab_bringup jetson-arlab2.launch.py
   ros2 launch arlab_bringup jetson-arlab3.launch.py
   ```

4. **Start decision maker manually on arlab-nuc**:

   ```bash
   ros2 run arlab_decision_making decision_maker --ros-args -p task:=...
   ```

## Launch file components

| Launch file | Main components |
| ----------- | --------------- |
| `arlab-nuc.launch.py` | MoveIt robot planning, manipulation nodes |
| `jetson-arlab1.launch.py` | Robot control, gripper camera calibration |
| `jetson-arlab2.launch.py` | speech output |
| `jetson-arlab3.launch.py` | computer vision processing (driver is a separate container) |
