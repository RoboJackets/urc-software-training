# Lesson 2 — Coordinate Transforms & TF2

> **Goal:** Understand coordinate frames, rotations, quaternions, and how ROS's
> **TF2** system tracks the relationship between every frame on the robot over
> time. This is the single most important concept for the rest of the course —
> localization, planning, and control are all about *frames*.

## Where this lives in the repo

- `src/robonav_training_common/include/robonav_training_common/angles.hpp` — the
  `normalizeAngle()` helper, used everywhere headings are compared.
- The TF tree is *produced* by several nodes you'll study later:
  - `robot_state_publisher` (from the URDF) → robot body frames.
  - `ekf_localization` → `odom` → `base_footprint`.
  - `particle_filter` → `map` → `odom`.
- `src/particle_filter/src/pf_tf_helpers.cpp` — a concrete example of *computing*
  and *broadcasting* a transform ("broadcasting" = publishing a transform; defined
  in the TF2 section below).

## Why frames exist

Every measurement on a robot is relative to *something*:

- The lidar reports "an obstacle 2 m ahead **of the lidar**."
- The wheel odometry says "I've moved 3 m **from where I started**."
- The map says "the goal is at (5, 1) **in the map**."

"Ahead of the lidar," "from where I started," and "in the map" are three different
**coordinate frames**. To use these together — e.g. "is that lidar obstacle on my
planned path?" — you must convert between frames. A **transform** is the recipe for
that conversion.

## A frame = position + orientation

A 2D frame is fully described by where its origin sits and how it's rotated:
`(x, y, θ)`. In 3D it's `(x, y, z)` plus an orientation. A **transform** between two
frames is exactly this triple: a **translation** (shift) plus a **rotation**.

ROS Convention states:

- **x** points forward, **y** points left, **z** points up ("right-handed").
- Angles follow the right-hand rule: positive yaw (θ) turns counter-clockwise
  (toward +y, i.e. to the left).

## Rotations in 2D

To rotate a point `(x, y)` by angle θ (the heart of half the code in this repo):

```
x' = x·cos θ − y·sin θ
y' = x·sin θ + y·cos θ
```

You'll see this exact pattern over and over. For example in
`pure_pursuit_core.cpp`, the lookahead point is rotated into the robot's frame with
the *inverse* rotation (note the sign flips):

```cpp
x_r =  cos(ryaw) * dx + sin(ryaw) * dy;   // forward axis
y_r = -sin(ryaw) * dx + cos(ryaw) * dy;   // left axis
```

And in `wheel_odometry.cpp`, motion in the body frame is rotated into the world
frame by the robot's heading. Same two lines, recur everywhere. **Learn to
recognize the rotation pattern.**

## Composing and inverting transforms

Transforms chain. If you know `map → odom` and `odom → base`, you get
`map → base` by composing them. If you know `odom → base`, you get `base → odom` by
**inverting**. The particle filter does exactly this in `pf_tf_helpers.cpp`:

```
map→odom = (map→base) · inverse(odom→base)
```

It knows where the robot is in the map (`map→base`, from the particle estimate) and
where the robot is in odom (`odom→base`, from odometry), and solves for the missing
link `map→odom`. You don't need to do this matrix algebra by hand — `tf2` does it —
but you should understand *that this is what's happening*.

(Transforms can be represented as rotation+translation matrices: the `·` is matrix
multiplication, read right-to-left, and
`inverse(odom→base)` is the matrix inverse, which flips `odom→base` into
`base→odom`.)

## 3D orientation: why quaternions

In 2D a single angle θ is enough. In 3D, orientation needs three numbers
(roll, pitch, yaw) — but representing them as three angles has nasty failure modes
("gimbal lock") and ambiguous ordering. So ROS stores 3D orientation as a
**quaternion**: four numbers `(x, y, z, w)`.

You rarely build quaternions by hand. Things to know:

- A quaternion encodes "an axis to spin around, and how much."
- For a flat robot, only yaw matters, so the quaternion is just yaw in disguise.
- Convert yaw → quaternion with `tf2::Quaternion::setRPY(0, 0, yaw)` (used in
  `wheel_odometry.cpp` and `map_loader.cpp`).
- Convert quaternion → yaw with the formula in `pure_pursuit.cpp`:

  ```cpp
  yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
  ```

That's the only quaternion math you need for this course: 2D yaw ⇄ quaternion.

## Angle wrapping — `normalizeAngle`

Angles are circular: +179° and −181° are the *same* heading. If you naively
subtract two headings you can get a "358° error" that should be "−2°." Every place
this repo compares or accumulates headings, it wraps the result to the equivalent
angle at the `-π`/`π` boundary (normally described as `[-π, π]`).

Open `src/robonav_training_common/include/robonav_training_common/angles.hpp`:

```cpp
namespace robonav_training {
inline double normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}
```

Why does `atan2(sin θ, cos θ)` work? `sin` and `cos` are periodic, so they throw
away whole turns; `atan2` then reconstructs the unique angle in `(−π, π]`. It's a
branch-free trick — no `if (angle > π) angle -= 2π;` loops.

This tiny function is used in:

- `wheel_odometry.cpp` — wrapping the integrated heading every step.
- `particle_filter` — wrapping odometry deltas and per-particle headings.
- `pure_pursuit_core.cpp` — wrapping the heading error to the lookahead point.

**Rule of thumb:** any time you add to or subtract angles, wrap the result.

## TF2: the system that tracks all frames

You could pass transforms around manually, but with a dozen frames updating at
different rates that's a nightmare. **TF2** is ROS's dedicated subsystem for this.

- Nodes **broadcast** transforms ("right now, `map→odom` is this") onto the special
  topics `/tf` (dynamic — transforms that change over time, like the moving robot)
  and `/tf_static` (fixed — transforms that never change, like the rigid URDF body).
  "Broadcast" is just the TF-specific word for "publish."
- TF2 stores a short time-history of every transform in a **buffer**.
- Any node can **look up** "what is the transform from frame A to frame B at time
  t?" and TF2 walks the tree and interpolates over time to answer.

This is why frames must form a **tree** (each frame has exactly one parent). If two
nodes both tried to publish `odom`'s parent, the tree would be ambiguous.

### The TF tree in this repo

```
map                         (the fixed world / the static map)
 └─ odom                    published by particle_filter  (map → odom)
     └─ base_footprint      published by ekf_localization (odom → base_footprint)
         └─ base_link       published by robot_state_publisher (from URDF)
             ├─ left_wheel_link
             ├─ right_wheel_link
             ├─ caster_link
             ├─ imu_link
             └─ sensor_tower_top_link → lidar_link
```

(Each of those *publisher* nodes gets its own lesson — `robot_state_publisher` in
Lesson 3, the EKF in Lesson 7, the particle filter in Lesson 9. Don't worry about
what they are yet; just see that **different nodes own different edges of the
tree**.)

Read this top-to-bottom as "less certain, slower-changing" → "more certain,
faster-changing":

- **`map → odom`** (from the **particle filter**, Lesson 9): the *correction*. It
  jumps occasionally to snap the robot onto the map. Drift-free but discontinuous.
- **`odom → base_footprint`** (from the **EKF**, Lesson 7): smooth, continuous
  dead-reckoning. Never jumps, but slowly drifts.
- **`base_footprint → base_link → sensors`** (from **robot_state_publisher**,
  Lesson 3): rigid, fixed by the URDF — the robot's physical shape.

This `map / odom / base_link` split is a ROS standard (REP 105). The genius of it:
controllers get a *smooth* pose from `odom`, while the slow `map→odom` correction
keeps the whole thing globally accurate — and no single transform has to be both
smooth and drift-free.

### Inspect TF from the terminal

```sh
ros2 run tf2_tools view_frames        # writes a PDF diagram of the whole tree
ros2 run tf2_ros tf2_echo map base_link   # live transform between two frames
ros2 topic echo /tf                   # raw dynamic transforms
ros2 topic echo /tf_static            # fixed transforms (the URDF ones)
```

In RViz, add a **TF** display to see every frame drawn as little axis triads (sets
of red/green/blue arrows, one per frame) — the best way to build intuition. The
RViz **Fixed Frame** setting (top-left of RViz) chooses which frame is treated as
the stationary world; everything else is drawn relative to it. Watch
`base_footprint` glide (odom) and the whole robot occasionally hop (map correction).

## How a transform is broadcast (peek at the code)

`pf_tf_helpers.cpp` (Lesson 9 covers the math) shows the mechanics: build a
`geometry_msgs::TransformStamped`, set `header.frame_id = "map"` (parent),
`child_frame_id = "odom"` (child), fill translation + quaternion, and call
`tf_broadcaster_->sendTransform(...)`. That's the whole pattern — every TF
publisher does this.

## Hands-on

At this point the simulator publishes only the rigid body frames
`base_footprint → base_link → sensors`. Start it with
`ros2 launch robonav_training_bringup sim.launch.py`, then:

1. `ros2 run tf2_tools view_frames` — it writes `frames.pdf` in the current folder.
   Open it; you'll see the `base_footprint → base_link → … → lidar_link` body tree.
2. `ros2 run tf2_ros tf2_echo base_link lidar_link` — note the transform is constant
   (it's a rigid body, fixed by the URDF).
3. In RViz, enable the **TF** display and identify `base_footprint`, `base_link`,
   and `lidar_link`. Lesson 7 adds `odom`; Lesson 9 adds `map`.

## Video supplement

- **Articulated Robotics — "Understanding ROS 2 transforms (TF2)** (Articulated Robotics)
- **3Blue1Brown — "Quaternions" / "Visualizing quaternions"** for deep intuition
  on why 3D rotation is weird (optional, math-heavy).
- Official: <https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html>

## Check yourself

- What three numbers describe a 2D frame?
- Write the 2D rotation formula from memory. Where does it appear in
  `pure_pursuit_core.cpp`?
- Why does `normalizeAngle` use `atan2(sin θ, cos θ)`?
- Why is the pose split into `map→odom` (jumpy) and `odom→base` (smooth) instead of
  one transform?
- Why must TF frames form a tree?

## Recap

Everything on a robot lives in a **frame**; a **transform** (translation +
rotation) converts between frames; 2D rotation is two lines of trig, 3D
orientation is a **quaternion** (for us, just yaw in disguise); angles must be
**wrapped** with `normalizeAngle`. **TF2** stores the time-history of the whole
frame **tree** so any node can look up any relationship. The repo's tree is
`map → odom → base_footprint → base_link → sensors`, split exactly so the pose can
be both smooth and globally accurate.

➡️ **Next:** [Lesson 3 — URDF & the Robot Description](lesson-03-urdf-robot-description.md),
where those body frames actually come from.
