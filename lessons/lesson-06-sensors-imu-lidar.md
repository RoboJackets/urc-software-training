# Lesson 6 — Sensors: IMU & Lidar

> **Goal:** Understand the two main sensors this robot carries — the **IMU** and the
> **lidar** — what they measure, what their messages contain, why they're noisy, and
> how simulated sensor data gets from Gazebo into ROS via the **bridge**.

## Where this lives in the repo

- `src/robonav_training_description/urdf/sensors/imu.xacro` — the IMU sensor.
- `src/robonav_training_description/urdf/sensors/lidar.xacro` — the lidar sensor.
- `src/robonav_training_bringup/config/gz_bridge.yaml` — the Gazebo↔ROS bridge.
- `src/robonav_training_bringup/worlds/empty.sdf`, `training_world.sdf` — the
  simulated worlds the sensors perceive.

## Two kinds of sensing

- **Proprioceptive** sensors measure the robot's *own* motion: the wheel encoders
  (Lesson 5) and the IMU's gyro/accelerometer.
- **Exteroceptive** sensors measure the *world*: the lidar.

Localization needs both — internal motion to predict where you went, external
measurements to correct drift. That's the EKF (next lesson) and the particle filter
(Lesson 9).

## The IMU

An **IMU** (Inertial Measurement Unit) combines:

- a **gyroscope** — measures **angular velocity** (rad/s) about each axis,
- an **accelerometer** — measures **linear acceleration** (m/s²), including gravity,
- (often) a fused **orientation** estimate.

Our IMU is defined in `imu.xacro` as a Gazebo `imu` sensor on `imu_link`, at
**100 Hz**, publishing `sensor_msgs/Imu` on **`/imu/data`**. It includes realistic
**Gaussian noise**: gyro σ ≈ 0.0017 rad/s, accelerometer σ ≈ 0.017 m/s² per axis.

### The `sensor_msgs/Imu` message

```
header                     # stamp + frame_id (imu_link)
orientation                # quaternion (3D heading estimate)
angular_velocity           # x, y, z  (rad/s)  ← gyro
linear_acceleration        # x, y, z  (m/s²)   ← accel (includes gravity!)
*_covariance               # how noisy each is
```

For our planar robot, the parts that matter are the **yaw** (heading) from
`orientation` and the **z angular velocity** (turn rate) from `angular_velocity`.
The EKF uses exactly those (Lesson 7).

### Why noise matters

Gravity (~9.81 m/s² down) dominates the accelerometer, and integrating noisy
acceleration twice to get position is hopeless — error explodes. So an IMU is
**not** used for position. Its gyroscope, however, gives an excellent short-term
turn rate, and its orientation gives a heading that doesn't drift the way wheel
odometry's heading does. The deliberate noise in the sim is what makes the EKF's
job realistic — fusion exists precisely because every sensor is imperfect.

## The lidar

A **lidar** ("light detection and ranging") spins a laser and measures the distance
to the nearest surface at many angles, producing a 2D "slice" of the room — a ring
of ranges.

Our lidar is defined in `lidar.xacro` as a Gazebo `gpu_lidar` on `lidar_link` (atop
the tower — Lesson 3), publishing `sensor_msgs/LaserScan` on **`/lidar/scan`**:

- **10 Hz**, **360 samples**,
- angle range **−90°…+90°** (`−1.5708 … +1.5708` rad) — a forward 180° fan,
- range **0.12 m … 12.0 m**.

### The `sensor_msgs/LaserScan` message

```
header            # stamp + frame_id (lidar_link)
angle_min         # angle of the first beam (rad)
angle_max         # angle of the last beam (rad)
angle_increment   # angle step between beams (rad)
range_min/max     # valid distance range (m)
ranges[]          # the measured distances, one per beam
```

To get the (x, y) of beam *i* in the lidar frame: its angle is
`angle_min + i · angle_increment`, and the hit point is
`ranges[i] · (cos angle, sin angle)`. The particle filter (Lesson 9) does exactly
this for thousands of beams to score "does this scan match the map from here?"

A beam reading `inf`/NaN or outside `[range_min, range_max]` means "no return" and
must be skipped — the particle filter handles that.

## Why simulate sensors at all?

It's faster, safer, and free to develop and test on a simulated robot than a real
one. Gazebo computes physically plausible lidar returns and IMU readings (with
noise) against the simulated world. Your navigation code can't tell the difference —
it just sees `/lidar/scan` and `/imu/data`. When you move to real hardware, you
swap the sensor drivers; the rest of the stack is unchanged. That portability is a
core reason ROS structures everything around topics.

## The Gazebo ↔ ROS bridge

Here's a subtlety: Gazebo (the modern "Ignition"/`gz` simulator) has its **own**
message system, separate from ROS 2. A Gazebo `LaserScan` is not a ROS
`LaserScan`. Something must translate between them — the **`ros_gz_bridge`**.

Open `gz_bridge.yaml`. Each entry maps one Gazebo topic to one ROS topic, giving
both type names and a direction:

```yaml
- ros_topic_name: "/lidar/scan"
  gz_topic_name:  "/lidar/scan"
  ros_type_name:  "sensor_msgs/msg/LaserScan"
  gz_type_name:   "ignition.msgs.LaserScan"
  direction: GZ_TO_ROS
```

The three bridged topics in this repo:

| ROS topic | Type | Direction | Consumed by |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | GZ → ROS | everything (`use_sim_time`) |
| `/lidar/scan` | `sensor_msgs/LaserScan` | GZ → ROS | particle filter |
| `/imu/data` | `sensor_msgs/Imu` | GZ → ROS | EKF |

`/clock` is why `use_sim_time` (Lesson 0) works: Gazebo publishes simulation time,
the bridge forwards it to ROS, and every node with `use_sim_time: true` uses it
instead of the wall clock — so logs, TF, and sensor stamps all line up even if the
sim runs faster or slower than real time.

> Note: `/cmd_vel` and `/joint_states` are **not** in the bridge — they go through
> `ros2_control`'s Gazebo plugin (Lesson 3), a different path. Only the raw sensors
> and the clock use the parameter bridge.

## The simulated world

The sensors perceive whatever is in the loaded SDF world.

- `empty.sdf` — just a ground plane, gravity, and light. Good for teleop and
  controller tuning (no obstacles for the lidar to see).
- `training_world.sdf` — has obstacles arranged to **match** the static map in
  `map_server/maps/` (Lesson 8). That match is what makes localization work: the
  lidar sees walls that exist in the map, so the particle filter can line them up.

`sim.launch.py` loads `training_world.sdf` by default (override with the `world`
launch argument).

## Hands-on

No new package is required for this lesson. Use the provided simulator, sensor
URDF, and bridge configuration as the data source for the EKF and particle filter
you implement later.

With the sim running:

```sh
# Confirm the bridged sensors are alive and at the right rates:
ros2 topic hz /imu/data        # ~100 Hz
ros2 topic hz /lidar/scan      # ~10 Hz
ros2 topic echo /imu/data --once
ros2 topic echo /lidar/scan --once   # look at angle_min, angle_increment, ranges[]

# Check sim time is flowing:
ros2 topic echo /clock --once
```

In RViz:

1. Add a **LaserScan** display on `/lidar/scan`. Set RViz's **Fixed Frame** to
   `base_link` (`odom` is added in Lesson 7 and `map` in Lesson 9). Drive around
   and watch the red points trace the walls.
2. Note how the scan is a forward 180° fan, not a full circle — that matches the
   `−90°…+90°` config from `lidar.xacro`.
3. Try launching with the packaged empty world and see the lidar return nothing —
   no walls to hit:

   ```sh
   ros2 launch robonav_training_bringup sim.launch.py \
     world:=$(ros2 pkg prefix robonav_training_bringup)/share/robonav_training_bringup/worlds/empty.sdf
   ```

## Video supplement

- **What is an Inertial Measurement Unit (IMU)?** by Phidgets Inc.
- **What Is LiDAR and how does LiDAR work?** by Phoenix LiDAR Systems

## Check yourself

- What does a gyroscope measure vs. an accelerometer? Why can't you get position
  from an accelerometer?
- Which fields of `sensor_msgs/Imu` does our planar robot actually care about?
- Given a `LaserScan`, how do you compute the (x, y) of the *i*-th beam's hit?
- Why is a Gazebo→ROS bridge necessary, and which three topics does it carry here?
- Why must `training_world.sdf` match the map used by the particle filter?

## Recap

The robot senses its motion with an **IMU** (`/imu/data`, 100 Hz, gyro + accel +
orientation, noisy) and the world with a **lidar** (`/lidar/scan`, 10 Hz, a forward
180° fan of ranges). Both are simulated in Gazebo and carried into ROS by the
**`ros_gz_bridge`** (configured in `gz_bridge.yaml`), along with `/clock` for
`use_sim_time`. The IMU feeds the EKF; the lidar feeds the particle filter; the
world must match the map. Next, fuse the IMU with wheel odometry.

➡️ **Next:** [Lesson 7 — EKF & Sensor Fusion](lesson-07-ekf-sensor-fusion.md).
