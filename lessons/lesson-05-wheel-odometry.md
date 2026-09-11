# Lesson 5 — Wheel Odometry

> **Goal:** Build the first substantial robotics node, `wheel_odometry`: write the
> composable C++ node, subscribe to encoders, integrate pose, publish
> `/wheel/odometry`, and understand precisely why and how it drifts.

## What you will create

- `src/wheel_odometry/src/wheel_odometry.cpp` — the implementation.
- `src/wheel_odometry/include/wheel_odometry/wheel_odometry.hpp` — class + state.
- `src/wheel_odometry/launch/wheel_odometry.launch.py` — runs its generated
  standalone executable; Lesson 12 loads the same class as a component.
- Uses `robonav_training_common/angles.hpp` (`normalizeAngle`, Lesson 2).

In your starter repo, this package is intentionally absent. Follow
[Lesson 5 implementation steps](implementation-steps/lesson-05-wheel-odometry.md)
after reading the concept sections below; those steps create the package before
anything references it from CMake or launch.

## What odometry *is*

**Odometry** is estimating your change in position by counting your own motion —
here, by integrating wheel rotations. It's also called **dead reckoning**. The node
turns a stream of wheel encoder readings into a continuously updated pose estimate
`(x, y, θ)` and velocity `(v, ω)`, published as a `nav_msgs/Odometry` message.

## The node at a glance

You will write `WheelOdometry` as a `rclcpp::Node` and register it as a composable
component.

**Parameters** (declared in the constructor, with defaults):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `wheel_radius` | `0.075` | wheel radius r (m) — matches the URDF |
| `wheel_separation` | `0.34` | distance between wheels L (m) — matches the URDF |
| `left_joint_name` | `"left_wheel_joint"` | which joint in `/joint_states` is the left wheel |
| `right_joint_name` | `"right_wheel_joint"` | which joint is the right wheel |

**Subscribes:** `/joint_states` (`sensor_msgs/JointState`) — the encoder feed from
`joint_state_broadcaster` (Lesson 3), queue depth 10, callback `onJointState()`.

**Publishes:** `/wheel/odometry` (`nav_msgs/Odometry`) with
`header.frame_id = "odom"` and `child_frame_id = "base_footprint"`.

**Does NOT publish TF.** It only publishes the odometry *message*. The EKF
(Lesson 7) consumes it and owns the `odom → base_footprint` transform. This is the
"one parent per frame" rule from Lesson 2 in action.

## State the node carries between messages

From the header, the node needs to remember:

- the integrated pose `(x, y, theta)`
- whether it has seen a first valid encoder message yet
- the previous left/right wheel positions in radians
- the previous message timestamp

Odometry is inherently *incremental*: each update needs the **previous** wheel
positions and time to compute a delta. Hence all this remembered state.

## The first message: establish a baseline

On the very first valid `/joint_states` (one that contains both wheel joints), the
node can't compute a delta yet — there's nothing to subtract from. So it:

1. records the left and right wheel positions from the current message,
2. records the message timestamp,
3. marks that the baseline exists,
4. publishes a zero pose/zero velocity odometry at the origin,
5. returns without integrating.

Every subsequent message does the real work.

## The integration step

Inside `onJointState()`, once a baseline exists, you implement the Lesson 4 math:

1. Convert wheel-position deltas from radians to ground distance with `d = r * delta`.
2. Compute center distance and heading change.
3. Integrate pose with the midpoint heading, not the old heading.
4. Normalize the updated heading.
5. Divide by elapsed time to get forward speed and turn rate.

Ignore a sample when `dt <= 0.0`; two messages can share a timestamp or arrive out
of order, and integrating either one would corrupt both pose and the encoder
baseline.

Finally the callback updates the stored previous wheel positions and timestamp for
next time, then calls `publishOdometry()`.

In this version of the exercise, keep the integration math directly in
`wheel_odometry.cpp`. The implementation steps provide the surrounding ROS
structure, and you verify the behavior with the acceptance commands below.

## Filling the Odometry message

`publishOdometry()` builds the `nav_msgs/Odometry`:

- `header.stamp` = the joint-state timestamp; `header.frame_id = "odom"`.
- `pose.pose.position.x/y` = `x_`, `y_`; `z = 0`.
- `pose.pose.orientation` = quaternion from `theta_` via `setRPY(0, 0, theta_)`
  (Lesson 2: yaw → quaternion; roll = pitch = 0 because we're planar).
- `twist.twist.linear.x = vx`, `twist.twist.angular.z = wz`; all other components 0.
- A simple diagonal **covariance**. A *covariance* here is a matrix of
  uncertainties attached to the estimate — one number per variable saying "how
  unsure am I about this?" (bigger = less confident). "Diagonal" means we only fill
  in the per-variable variances (x, y, yaw) and leave the cross-terms zero. This
  node sets small values for x/y (`1e-3`) and a larger one for yaw (`1e-2`) — it's
  the node *declaring how much to trust itself*. That declaration matters enormously
  in Lesson 7, where the EKF weighs each input by its stated uncertainty. (Full
  treatment of covariance is in Lesson 7.)

## Why pose vs. twist matters for the EKF (foreshadowing)

Look back at the message: it reports both an **absolute pose** (x, y, θ) and
**velocities** (v, ω). The EKF config (`ekf.yaml`, Lesson 7) deliberately fuses
**only the velocities** from this topic, *not* the absolute pose. Why? Because the
integrated pose **drifts** — its error grows without bound — while the *velocities*
are trustworthy at each instant. Hold that thought; it's the key design decision in
the next lesson.

## Why it drifts (the whole point)

Each step adds a tiny error (wheel slip, imperfect r/L, encoder quantization,
timing jitter). The integration *sums* those errors:

- Heading error is worst: a small constant `Δθ` bias rotates the whole future path,
  so position error grows roughly with distance traveled.
- There is **no correction** — nothing in this node ever compares against the
  outside world. It can only ever accumulate.

So wheel odometry is excellent **short-term** and **smooth** (great for control),
but **unbounded** long-term. That's precisely the role it plays in the stack: it
provides the smooth `odom` motion, and other layers (IMU fusion, lidar map
matching) supply the bounded global correction.

## Hands-on

First complete
[Lesson 5 implementation steps](implementation-steps/lesson-05-wheel-odometry.md).

Wheel odometry needs the simulator (for `/joint_states`) but **not** the full
navigation stack — its own launch file is enough. Use a separate sourced terminal
inside TigerVNC for each command below (or separate `docker compose exec` shells
from your host; see the README's "How to do the hands-on"):

```sh
# Terminal 1 — the simulator (gives us /joint_states):
ros2 launch robonav_training_bringup sim.launch.py

# Terminal 2 — run wheel odometry on its own:
ros2 launch wheel_odometry wheel_odometry.launch.py

# Terminal 3 — watch its output:
ros2 topic echo /wheel/odometry # Read the instructions below if the output doesn't fit on your screen

# Terminal 4 — drive:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**If you only see 0.0 printing out after `ros2 topic echo /wheel/odometry`, don't panic. The `nav_msgs::msg::Odometry` message prints out covariance (in this case as 36 '0.0' values) for both the pose and the twist. You don't have to worry about covariance now but if it makes it difficult to view the pose and twist information you can print the pose and twist directly using the commands below.**

```sh
# Terminal 3 - Print out just the pose
ros2 topic echo /wheel/odometry --field pose.pose

# Terminal 3 - Print out just the twist
ros2 topic echo /wheel/odometry --field twist.twist

```

Experiments:

1. Drive forward 1 m (watch `pose.position.x` climb). Drive back. Does it return to
   ~0, or is there leftover error? That's drift.
2. Spin in place several full turns, then stop. Drive forward. Notice how a small
   heading error now sends "forward" in the wrong direction — heading drift is the
   killer.
3. Set a deliberately wrong `wheel_separation`. The node also installs a standalone
   executable, so run it directly with a parameter override (instead of the launch
   file, which doesn't take `-p`): `ros2 run wheel_odometry wheel_odometry
   --ros-args -p wheel_separation:=0.30`. Drive a square and watch the path fail to
   close — this is why the URDF and odometry constants must match.

## Video supplement

- [Modern Robotics, Chapter 13.4: Odometry (Northwestern Robotics)](https://www.youtube.com/watch?v=eQ9E0Zvp9jw)

## Check yourself

- Why must the first message only establish a baseline and not integrate?
- Map each line of the integration code back to a Lesson 4 formula.
- Why does the node publish a message but *not* a TF?
- The message contains both pose and twist — which does the EKF use, and why?
- Explain, in terms of accumulation, why heading drift hurts more than position
  drift.

## Recap

`wheel_odometry` subscribes to `/joint_states`, converts wheel-radian deltas to
ground distances (`d = rΔφ`), applies diff-drive forward kinematics and midpoint
integration to maintain `(x, y, θ)`, derives `(v, ω)`, and publishes
`nav_msgs/Odometry` on `/wheel/odometry` with a self-declared covariance. It's
smooth and accurate short-term but drifts without bound — which is exactly why the
EKF fuses its *velocities* with an IMU next.

➡️ **Next:** [Lesson 6 — Sensors: IMU & Lidar](lesson-06-sensors-imu-lidar.md).
