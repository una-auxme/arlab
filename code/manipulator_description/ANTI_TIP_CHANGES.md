# Anti-tip changes for mobile Gazebo simulation

Changed files:

- `manipulator_description/urdf/manipulator_mobile.urdf.xacro`
- `manipulator_description/urdf/ur_chasis.xacro`

## What changed

### 1. Root base link is locked against physics tilt

In the mobile URDF, `ranger_base_link` now has Gazebo settings:

```xml
<gazebo reference="ranger_base_link">
  <gravity>false</gravity>
  <kinematic>true</kinematic>
  <self_collide>false</self_collide>
</gazebo>
```

Reason: the base is already moved kinematically by `gz-sim-velocity-control-system` through `/cmd_vel`. Therefore Gazebo does not need to solve physical roll/pitch dynamics for the base. The robot can still move in x/y/yaw, but the base should no longer tip during driving.

### 2. Invisible anti-tip ballast added

A hidden `anti_tip_ballast_link` is fixed to `ranger_base_link`. It has no visual or collision geometry. It only changes the inertial behaviour and lowers/stiffens the combined centre of mass. This is a fallback in case a Gazebo version ignores the `kinematic` tag during URDF-to-SDF conversion.

### 3. Chassis inertial model moved down

The previous chassis inertial origin was at `z=0.5`, which made the robot unnecessarily top-heavy in Gazebo. It is now at `z=0.12`, with increased mass/inertia for numerical stability.

## How to disable the lock for testing

The mobile URDF has this xacro argument:

```xml
<xacro:arg name="lock_base_tilt" default="true" />
```

For normal mobile simulation, keep it `true`. For a physics-only comparison, generate the robot with:

```bash
lock_base_tilt:=false
```

