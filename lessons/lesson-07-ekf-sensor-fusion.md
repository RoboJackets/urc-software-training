# Lesson 7 — EKF & Sensor Fusion

> **Goal:** Understand Bayesian filtering and the Extended Kalman Filter intuitively,
> then build the `ekf_localization` config package and launch file that fuse wheel
> odometry and the IMU into one smooth `/odometry/filtered` estimate plus the
> `odom → base_footprint` transform.

## What you will create

- `src/ekf_localization/config/ekf.yaml` — **all** of the configuration (this is a
  config-only package).
- `src/ekf_localization/launch/ekf_localization.launch.py` — launches the standard
  `robot_localization` `ekf_node` with that config.
- `src/ekf_localization/package.xml` — depends on `robot_localization`.

In your starter repo, this package does not need to exist before this lesson. Follow
[Lesson 7 implementation steps](implementation-steps/lesson-07-ekf-localization.md)
after reading the filter and configuration sections below.

There is no C++ in this package. It configures a battle-tested, reusable EKF from
the `robot_localization` package (installed in the Dockerfile, Lesson 1). Writing
your own EKF from scratch is a great exercise, but in practice you configure a
proven one — and learning to configure it correctly *is* the skill.

## The problem: every sensor is wrong, differently

From the last two lessons:

- **Wheel odometry** gives a smooth, accurate *velocity* and a heading that
  **drifts** over time (Lesson 5).
- **The IMU** gives an excellent *turn rate* and an *absolute heading* that doesn't
  drift the same way, but its acceleration is too noisy for position (Lesson 6).

Neither alone is enough. **Sensor fusion** combines them so the result is better
than either: use the wheels for forward speed, the IMU for heading, and let a
principled filter weigh each by its uncertainty.

## Bayesian filtering in one idea

A **Bayes filter** maintains a *belief* about the state (here: pose + velocity) and
updates it in two repeating steps:

1. **Predict** — use a motion model to advance the belief ("I was here, I moved
   roughly this much, so now I'm probably about here"). Prediction **increases
   uncertainty** (motion is imperfect).
2. **Update (correct)** — fold in a new measurement, pulling the belief toward what
   the sensor says, weighted by how much you trust the sensor. Update **decreases
   uncertainty**.

The belief is a Gaussian: a mean (best estimate) plus a **covariance** (how unsure
you are, per variable). This same predict/update skeleton is the *entire* idea
behind both the EKF here and the particle filter in Lesson 9 — they're two ways to
implement a Bayes filter.

## What the (Extended) Kalman Filter adds

A **Kalman Filter** is the optimal Bayes filter when everything is linear and noise
is Gaussian. It tracks the mean and covariance and computes a **Kalman gain** —
literally "how much to trust the new measurement vs. the prediction," derived from
their relative covariances. Confident measurement → trust it more; noisy
measurement → trust the prediction more.

Robot motion isn't linear (sines and cosines of heading), so we use the
**Extended** Kalman Filter (EKF): each step it **linearizes** the motion/measurement
models at the current estimate — i.e. it approximates the curved motion with a
straight-line (tangent) model that's accurate enough over one small time step — then
applies the Kalman update. For our purposes the intuition is identical — predict,
then correct, weighting by covariance.

This is exactly why `wheel_odometry` bothered to publish a **covariance**
(Lesson 5): the EKF reads those numbers to decide how much to trust each input.

## The state vector and `two_d_mode`

`robot_localization`'s EKF tracks a 15-element state: position (x, y, z), orientation
(roll, pitch, yaw), their velocities, and linear accelerations. Our robot is planar,
so `ekf.yaml` sets:

```yaml
two_d_mode: true
```

This forces z, roll, pitch, and their rates to zero — the filter effectively tracks
just `x, y, yaw, vx, vy, vyaw`. It prevents nonsense states like the robot tilting
or sinking through the floor and keeps the filter simpler and more stable. (The
configuration masks below still list all 15 slots; in `two_d_mode` the out-of-plane
ones are simply ignored even if you set them `true`.)

## Writing `ekf.yaml`

### Global settings

```yaml
frequency: 30.0          # run the predict/update loop at 30 Hz
sensor_timeout: 0.2      # if a sensor goes silent > 0.2 s, stop using it
two_d_mode: true         # planar robot (see above)
publish_tf: true         # broadcast the odom → base_footprint transform
```

### Frames

```yaml
map_frame: map
odom_frame: odom
base_link_frame: base_footprint
world_frame: odom
```

The crucial line is `world_frame: odom`. It means **this EKF operates in the `odom`
frame** and therefore publishes the **`odom → base_footprint`** transform. It is
*not* responsible for the map — that correction (`map → odom`) is the particle
filter's job (Lesson 9). This is the REP-105 split from Lesson 2, realized in
config: the EKF gives you smooth, continuous local odometry; the particle filter
gives the discrete global correction.

### How to read a `*_config` mask

Each sensor gets a 15-element boolean list — the same 15 state variables, in this
fixed order:

```
[ x,     y,      z,        # position
  roll,  pitch,  yaw,      # orientation
  vx,    vy,     vz,       # linear velocity
  vroll, vpitch, vyaw,     # angular velocity
  ax,    ay,     az ]      # linear acceleration
```

`true` in a slot = "fuse this variable from this sensor"; `false` = "ignore this
variable from this sensor." That's the whole mechanism — picking which sensor is
trusted for which part of the state.

### Input 1 — wheel odometry (`odom0`)

```yaml
odom0: /wheel/odometry
odom0_config: [false, false, false,    # x,    y,    z
               false, false, false,    # roll, pitch, yaw
               true,  false, false,    # vx,   vy,   vz
               false, false, true,     # vroll,vpitch,vyaw
               false, false, false]    # ax,   ay,   az
odom0_differential: false
odom0_relative: false
```

The 15-element `odom0_config` mask says **which variables to fuse** from this topic.
For the wheels we take **only `vx` and `vyaw`** — forward speed and turn rate. We
deliberately **ignore the absolute position and heading** from `/wheel/odometry`.

Why? Because (Lesson 5) the wheel odometry's integrated *pose* drifts without
bound, but its *instantaneous velocities* are trustworthy. Feeding the drifting
position into the filter would just import the drift. Feeding only velocities lets
the EKF do its own, better integration and lets heading come from a better source.

### Input 2 — the IMU (`imu0`)

```yaml
imu0: /imu/data
imu0_config: [false, false, false,     # x,    y,    z
              false, false, true,      # roll, pitch, yaw   ← take yaw
              false, false, false,     # vx,   vy,   vz
              false, false, true,      # vroll,vpitch,vyaw  ← take yaw rate
              false, false, false]     # ax,   ay,   az
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

From the IMU we take **absolute `yaw`** and **`vyaw`** (yaw rate). The IMU is the
robot's best heading source, so it supplies the orientation the wheels were told to
skip. `imu0_remove_gravitational_acceleration: true` subtracts the ~9.81 m/s²
gravity vector so it doesn't pollute the filter (we're not fusing acceleration
anyway, but it's correct hygiene).

### The fusion logic, summarized

| Source | Fuses | Provides | Why |
| --- | --- | --- | --- |
| `/wheel/odometry` | `vx`, `vyaw` | forward speed, turn rate | velocities are reliable; its position drifts |
| `/imu/data` | `yaw`, `vyaw` | absolute heading, turn rate | best heading; doesn't drift like wheels |

Note both supply `vyaw` — the turn rate is **observed twice**, from two independent
sensors, so the filter's heading-rate estimate is robust. The EKF blends them by
covariance, integrates velocity into a smooth pose, and never lets a single noisy
reading jerk the estimate.

## What the EKF produces

- **Topic `/odometry/filtered`** (`nav_msgs/Odometry`) — the fused pose + velocity.
  This is what `pure_pursuit` (Lesson 11) and `particle_filter` (Lesson 9) use as
  the robot's smooth motion estimate.
- **TF `odom → base_footprint`** — the smooth, continuous, never-jumping transform
  that controllers rely on.

It still drifts slowly (it's still dead reckoning at heart — no external landmark).
The bounded global fix comes from the particle filter correcting `map → odom`.

## How it's launched

You will write `ekf_localization.launch.py` to start `robot_localization`'s
`ekf_node` (named `ekf_filter_node`), load `ekf.yaml`, and set `use_sim_time`. In
the full stack it runs as its own process, separate from the component container
introduced in Lesson 12.

## Hands-on

First complete
[Lesson 7 implementation steps](implementation-steps/lesson-07-ekf-localization.md).

You can run the EKF without the full nav stack — just the simulator, wheel
odometry, and the EKF, each in its own sourced TigerVNC terminal or host-terminal
`docker compose exec` shell:

```sh
# Terminal 1 — simulator:
ros2 launch robonav_training_bringup sim.launch.py
# Terminal 2 — wheel odometry (the EKF's odom input):
ros2 launch wheel_odometry wheel_odometry.launch.py
# Terminal 3 — the EKF itself:
ros2 launch ekf_localization ekf_localization.launch.py
# Terminal 4 — inspect:
ros2 topic echo /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint   # the EKF's transform
```

Experiments:

1. Drive in a circle and compare `/wheel/odometry` vs `/odometry/filtered`. The
   filtered heading should track better and be smoother.
2. Watch `pose.covariance` in `/odometry/filtered` stay finite and change as the
   robot moves. It may grow during uncertainty or settle when measurements keep
   correcting the estimate.
3. Open `ekf.yaml` and flip the IMU's `yaw` to `false`, then **relaunch** the EKF
   (no `colcon build` needed — YAML is symlink-installed, Lesson 1) and drive. Watch
   heading get worse — that's the IMU's contribution made visible. (Put it back.)
4. In RViz set Fixed Frame to `odom`; the robot moves smoothly with no jumps — the
   EKF guarantees that continuity.

## Video supplement

- [Visually Explained: Kalman Filters (Visually Explained)](https://www.youtube.com/watch?v=IFeCIbljreY)
- [The Extended Kalman Filter (EKF): Why Taylor Expansions are Awesome (James Han)](https://www.youtube.com/watch?v=9X3jGGnbcvU)
- `robot_localization` docs: look up the ROS 2 state-estimation node parameters and
  sensor config vectors for the distro used by the container.

## Check yourself

- Describe the predict and update steps of a Bayes filter and what each does to
  uncertainty.
- What does the Kalman gain decide, and what determines it?
- Why does the EKF fuse only `vx`/`vyaw` from the wheels but `yaw`/`vyaw` from the
  IMU?
- What does `world_frame: odom` make this EKF responsible for, and what does it
  deliberately *not* handle?
- Why does the EKF still drift, and what fixes that later?

## Recap

Sensor fusion combines complementary, imperfect sensors via a **Bayes filter**:
**predict** with a motion model, **update** with measurements, weighting by
**covariance**. The **EKF** does this for nonlinear robot motion. This repo
configures `robot_localization`'s EKF (`ekf.yaml`) to fuse **wheel velocities** +
**IMU heading/turn-rate**, producing the smooth `/odometry/filtered` and the
`odom → base_footprint` transform. It's drift-bounded only locally — the particle
filter supplies the global correction next.

➡️ **Next:** [Lesson 8 — Occupancy Grids & the Map Server](lesson-08-occupancy-grids-map-server.md).
