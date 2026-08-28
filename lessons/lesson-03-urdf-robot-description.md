# Lesson 3 — URDF & the Robot Description

> **Goal:** Learn how a robot's physical shape, frames, sensors, and motors are
> described to ROS using **URDF** (written with **xacro**), and read this repo's
> actual robot description line by line.

## Where this lives in the repo

- `src/robonav_training_description/urdf/robonav_training_robot.urdf.xacro` — the
  main robot description.
- `src/robonav_training_description/urdf/sensors/imu.xacro` — the IMU.
- `src/robonav_training_description/urdf/sensors/lidar.xacro` &
  `lidar_tower.xacro` — the lidar and the tower it sits on.
- `src/robonav_training_description/config/ros2_control.yaml` — the controllers.
- `src/robonav_training_description/CMakeLists.txt`, `package.xml`.

## What is URDF?

**URDF** = Unified Robot Description Format. It's an XML document that tells ROS:

- the robot's **links** (rigid bodies — chassis, wheels, sensors),
- the **joints** that connect links (and whether/how they move),
- the **visual** shape (for RViz/Gazebo) and **collision** shape (for physics),
- the **inertial** properties (mass + how mass is distributed) for the simulator,
- where **sensors** are mounted and what they output.

From the URDF, `robot_state_publisher` produces the bottom of the TF tree you saw
in Lesson 2 (`base_footprint → base_link → wheels/sensors`). So URDF is literally
where the robot's frames come from.

### Links and joints

- A **link** is a rigid body. It has a name (which is also a TF frame name) and
  up to three descriptions: `<visual>`, `<collision>`, `<inertial>`.
- A **joint** connects a **parent** link to a **child** link and defines where the
  child sits relative to the parent (`<origin>`) and how it can move (`<type>`):
  - `fixed` — rigid, no motion (sensor mounts, the tower).
  - `continuous` — rotates without limit about an axis (the wheels).
  - (others: `revolute`, `prismatic` — not used here.)

Every joint's `<origin>` is a transform — so the joint tree *is* the lower TF tree.

## Why xacro?

Writing raw URDF means repeating yourself constantly (two nearly identical wheels,
four identical tower legs). **xacro** ("XML macros") adds variables, math, and
reusable macros, then expands to plain URDF at launch. You'll see:

- **Properties** (constants): `<xacro:property name="wheel_radius" value="0.075"/>`
- **Macros** (reusable chunks): `<xacro:macro name="wheel_inertial" params="...">`
- **Includes**: `<xacro:include filename="sensors/lidar.xacro"/>`
- **Math** in attributes: `${base_height/2 + 0.002}`

This repo uses xacro properties so you can re-tune the robot (wheel size, mass,
separation) in one place — which is exactly what you'd adjust when calibrating
odometry later.

## This robot, part by part

Read `robonav_training_robot.urdf.xacro` alongside this. It's a differential-drive
robot: two powered wheels + a passive caster, with a lidar on a tower and an IMU in
the body.

### The base frames

- **`base_footprint`** — a virtual frame on the ground directly under the robot,
  at z = 0. This is the robot's "where am I on the floor" frame. Localization
  (EKF, particle filter) tracks *this* frame.
- **`base_footprint_joint`** (fixed) raises up by **0.12 m** to…
- **`base_link`** — the chassis: a box **0.42 × 0.30 × 0.12 m**, mass **8.0 kg**.
  (The 0.12 m joint offset and the 0.12 m box height are independent numbers that
  happen to be equal — the offset positions the body above the ground frame.)
  This is the robot's body and the parent of everything physical.

Why two base frames? `base_footprint` is the clean ground-projected reference;
`base_link` is the actual body that may sit at some height. Keeping them separate
is a ROS convention that makes 2D navigation math clean.

### The drive wheels (the important part for kinematics)

- **`left_wheel_joint`** (continuous): origin at x = −0.07, **y = +0.17**, rotates
  about the **y-axis**; child **`left_wheel_link`** — a cylinder radius **0.075 m**,
  width 0.04 m, mass 0.6 kg.
- **`right_wheel_joint`** (continuous): same but **y = −0.17**; child
  **`right_wheel_link`**.

Two numbers here drive *all* of Lessons 4–5:

- **wheel radius = 0.075 m** → converts wheel spin (radians) to ground distance.
- **wheel separation = 0.34 m** (the two wheels are 0.17 + 0.17 apart) → converts
  the difference in wheel motion into turning.

These same numbers appear again as parameters in `wheel_odometry` and in
`ros2_control.yaml`. They **must agree**, or odometry will be wrong.

### The caster

- **`caster_joint`** (fixed) at the front; child **`caster_link`** — a small
  sphere (radius 0.035 m) with deliberately **low friction** (μ₁ = μ₂ = 0.01 are
  Gazebo friction coefficients, set in the URDF's `<gazebo>` tags) so it slides
  freely and doesn't fight the drive wheels. A passive caster is the classic
  third contact point for a two-wheel robot.

### The sensor tower + lidar

- **`sensor_tower_mount_joint`** (fixed) puts the plate center about `0.286 m` above
  the ground via
  **`sensor_tower_top_link`**, held by four identical legs (built with the
  `tower_leg` macro — a good example of *why* macros exist: four legs, one
  definition).
- **`lidar_joint`** (fixed) puts **`lidar_link`** on top of the plate.

The lidar is mounted high on a tower so its beams clear the chassis and sweep the
room unobstructed. `lidar.xacro` defines a Gazebo `gpu_lidar` **sensor**, which
produces a Gazebo scan. The separate `ros_gz_bridge` converts that scan into a ROS
message (full sensor and bridge details are in Lesson 6):

- ROS receives `sensor_msgs/LaserScan` on **`/lidar/scan`**, frame `lidar_link`,
- **10 Hz**, **360 samples**, angle range **−90°…+90°** (−1.5708…+1.5708 rad),
- range **0.12 m … 12.0 m**.

The particle filter (Lesson 9) consumes exactly this.

### The IMU

- **`imu_joint`** (fixed) places **`imu_link`** in the body. Defined in `imu.xacro`
  with a Gazebo `imu` sensor:
  - publishes `sensor_msgs/Imu` on **`/imu/data`**, **100 Hz**,
  - with realistic Gaussian noise: gyro σ ≈ 0.0017 rad/s, accel σ ≈ 0.017 m/s².

The EKF (Lesson 7) fuses this with wheel odometry. The deliberate noise is what
makes fusion *necessary* and the simulation honest.

## ros2_control: making the wheels actually turn

A URDF describes shape; it doesn't make motors spin. **ros2_control** is the
framework that connects ROS commands to (real or simulated) actuators. The hardware
plugin is selected in `robonav_training_robot.urdf.xacro`; controller settings live
in `config/ros2_control.yaml`:

- **Hardware plugin:** `gz_ros2_control/GazeboSimSystem` — bridges control to the
  Gazebo simulation.
- **`controller_manager`** runs at **50 Hz** and loads two **controllers** (in
  ros2_control a "controller" is just a plugin the manager loads — not a person):
  - **`joint_state_broadcaster`** — *reads* the wheel joint positions/velocities
    and publishes them on **`/joint_states`**. (This is the encoder feed that
    `wheel_odometry` integrates — Lesson 5.)
    > ⚠️ **Name clash:** a `joint_state_broadcaster` is a read-only *controller* that
    > publishes a **topic** (`/joint_states`). It is **not** the same thing as a TF
    > *broadcaster* (Lesson 2), which publishes **transforms**. ros2_control just
    > reuses the word "broadcaster" for "a controller that only reports state."
  - **`diff_drive_controller`** — *accepts* velocity commands and spins the two
    wheels accordingly. Configured with:
    - `left_wheel_names: ["left_wheel_joint"]`,
      `right_wheel_names: ["right_wheel_joint"]`,
    - **wheel_separation: 0.34**, **wheel_radius: 0.075** (matching the URDF!),
    - `base_frame_id: base_footprint`,
    - velocity limits ±0.7 m/s linear, ±2.5 rad/s angular,
    - command topic remapped so your **`/cmd_vel`** drives the robot.

So the full motor path is: someone publishes **`/cmd_vel`** → `diff_drive_controller`
→ wheel velocities in Gazebo → robot moves → `joint_state_broadcaster` publishes
**`/joint_states`** → `wheel_odometry` reads them back. That loop is the spine of
the next two lessons.

> **Note:** in this stack the **EKF** owns the `odom → base_footprint` transform, so
> `diff_drive_controller` is configured to publish odometry data but **not** to
> broadcast that TF (to avoid two nodes fighting over the same frame — Lesson 2's
> "one parent" rule).

## How the URDF gets loaded

In `sim.launch.py` (introduced in Lesson 1): the xacro is expanded to URDF and handed to
`robot_state_publisher` as the `robot_description` parameter; that node publishes
the static body transforms. A separate **spawn** step (the Gazebo command that
inserts a robot model into the running simulated world) injects the robot, and the
controllers above are started.

## Hands-on

Before answering, make this lesson hands-on with the running simulator:

```sh
ros2 topic echo /joint_states --once
ros2 topic echo /cmd_vel --once   # run this before pressing a teleop key
```

Match the wheel joint names in `/joint_states` to the URDF, then compare the wheel
radius and separation in the URDF with `ros2_control.yaml`.

## Check yourself

- What's the difference between a link and a joint? Between `base_footprint` and
  `base_link`?
- Which two URDF numbers determine the diff-drive kinematics, and where else in the
  repo must they match?
- Why is the lidar on a tower? Why does the caster have near-zero friction?
- What does `joint_state_broadcaster` publish, and which node consumes it?
- Why does the EKF, not `diff_drive_controller`, broadcast `odom → base_footprint`?

## Recap

**URDF** (written with **xacro**) describes the robot's **links**, **joints**,
inertia, and **sensors**, which become the lower TF tree via
`robot_state_publisher`. This robot is a diff-drive base (wheels 0.075 m radius,
0.34 m apart) with a tower lidar (`/lidar/scan`) and a body IMU (`/imu/data`).
**ros2_control** turns `/cmd_vel` into wheel motion and publishes `/joint_states`,
closing the loop that the next lessons build on.

➡️ **Next:** [Lesson 4 — Differential-Drive Kinematics](lesson-04-diff-drive-kinematics.md).
