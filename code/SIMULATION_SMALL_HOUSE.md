# Manipulator in `small_house` Gazebo world

This workspace now contains two pieces wired together:

- `small_house`: a small ROS 2 resource package that installs the Gazebo world and all `model://...` assets.
- `manipulator_description`: the robot description and launch files that spawn the manipulator into that world.

## Build

From the workspace root:

```bash
colcon build --packages-select small_house manipulator_description --symlink-install
source install/setup.bash
```

If dependencies such as `ur_description`, `ur_simulation_gz`, `ros_gz_sim`, `gz_ros2_control`, `mia_hand_description`, or `controller_manager` are missing, install/source those first or build the complete workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Start Gazebo with apartment + manipulator

```bash
ros2 launch manipulator_description manipulator.house.sim.launch.py
```

Useful arguments:

```bash
ros2 launch manipulator_description manipulator.house.sim.launch.py ur_type:=ur5e launch_rviz:=true
ros2 launch manipulator_description manipulator.house.sim.launch.py ur_type:=ur10e
ros2 launch manipulator_description manipulator.house.sim.launch.py gazebo_gui:=false
```

## Start full stack with MoveIt/RViz

```bash
ros2 launch manipulator_description manipulator.house.full.sim.launch.py
```

## What changed

1. `small_house` was converted into an installable ROS 2 package by adding `package.xml` and `CMakeLists.txt`.
2. `small_house/worlds/small_house.world` was added as installed world path.
3. The old TurtleBot include was removed from the world, because the manipulator launch spawns the robot instead.
4. `manipulator.house.sim.launch.py` sets `GZ_SIM_RESOURCE_PATH` and `IGN_GAZEBO_RESOURCE_PATH` so Gazebo can resolve all `model://aws_...` assets.
5. `manipulator.full.sim.launch.py` now forwards `world_file` and `gazebo_gui` to the lower Gazebo launch.
