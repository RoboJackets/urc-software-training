# Lesson 11 — Pure Pursuit Path Following

> **Goal:** Understand the pure pursuit path-tracking algorithm — the lookahead
> point and the curvature geometry — and implement the controller core that turns
> the planned `/plan` into `/cmd_vel` velocity commands.

## Starter files for this lesson

- `src/pure_pursuit/src/pure_pursuit_core.cpp` — the **pure** geometry (no ROS):
  `computePursuitCommand()`.
- `src/pure_pursuit/src/pure_pursuit.cpp` — the ROS wrapper: TF, the control timer,
  publishing `/cmd_vel`.
- `src/pure_pursuit/include/pure_pursuit/...` — headers.
- Uses `robonav_training_common`'s `normalizeAngle` (Lesson 2).

You implement the pure geometry function while the ROS wrapper can be provided as
starter code. Follow
[Lesson 11 implementation steps](implementation-steps/lesson-11-pure-pursuit.md)
after reading the algorithm sections below. In a starter repo, the files listed
above should already exist as scaffolding before you begin Lesson 11.

## Planning vs. control

Lesson 10's A* produced a **path** — a static list of waypoints. But a path is not
motion. **Control** (a.k.a. trajectory following) is the loop that, many times per
second, looks at where the robot *is* versus where the path *goes* and outputs a
velocity command to close the gap. Pure pursuit is a simple, robust, and extremely
popular controller for this.

## The pure pursuit idea

Imagine driving by always steering toward a point a fixed distance ahead of you on
the road — a "carrot on a stick." As you move, the target point slides forward along
the path, and you continuously curve toward it. That target is the **lookahead
point**, and the fixed distance is the **lookahead distance** `Ld`.

The robot then follows the unique **circular arc** that connects its current
position to the lookahead point. Pure pursuit reduces the whole path-following
problem to "find the lookahead point, compute the arc, drive it."

## The geometry (the one formula to know)

First, **curvature** is just `1 / radius`: a gentle arc (big circle) has small
curvature; a tight arc (small circle) has large curvature. A straight line has
curvature 0 (infinite radius).

Put the lookahead point into the **robot's own frame** (Lesson 2's inverse
rotation): `x_r` is forward, `y_r` is left. The arc from the robot to that point has
**curvature**:

```
curvature κ = 2 · y_r / Ld²
```

where `Ld = hypot(x_r, y_r)` is the actual distance to the lookahead point.

Why this formula? The robot (at the origin of its own frame) and the lookahead point
both lie on a circle that's tangent to the robot's forward direction. For such a
circle, the radius is `R = Ld² / (2·y_r)` — you can verify it: the circle's center
sits straight out to the side at `(0, R)`, and requiring the lookahead point
`(x_r, y_r)` to be exactly `R` away gives `x_r² + (y_r − R)² = R²`, which simplifies
to `R = (x_r² + y_r²) / (2·y_r) = Ld² / (2·y_r)`. Since curvature is `1/R`, that's
**κ = 2·y_r / Ld²**. Intuition:

- lookahead point straight ahead (`y_r = 0`) → curvature 0 → drive straight,
- lookahead point off to the left (`y_r > 0`) → curve left,
- the farther off-axis it is, the sharper the turn.

Curvature relates linear and angular velocity by `ω = v · κ`, so once you pick a
forward speed `v`, the turn rate falls right out. This respects the non-holonomic
diff-drive constraint from Lesson 4 — the robot only ever moves forward and turns,
never sideways.

## `computePursuitCommand()` step by step

The pure core returns:

```cpp
struct PursuitCommand { double linear; double angular; bool goal_reached; };
```

Given the robot pose, the path, and the tuning params, it:

1. **Goal check.** If the robot is within `goal_tolerance` of the final path point,
   return `{0, 0, goal_reached = true}` — stop.
2. **Find the nearest path point** to the robot (linear scan, min distance).
3. **Find the lookahead point**: search **forward only** from the nearest point for
   the first point at least `lookahead_distance` away. Searching forward-only
   reduces the chance of latching onto an earlier part of the path. If no point is
   far enough (near the end), use the final point.
4. **Transform the lookahead point into the robot frame** (`x_r`, `y_r`).
5. **Compute the command:**
   - If the lookahead point is **behind** the robot (`x_r < 0`): it has drifted
     behind, so crawl forward slowly and turn hard toward it (turn roughly in place)
     using the **sign** of `normalizeAngle(atan2(y_r, x_r))` to command
     `±max_angular_speed`.
   - Otherwise compute `κ = 2·y_r / Ld²`, set `linear = desired_linear_speed`, and
     `angular = clamp(linear · κ, ±max_angular_speed)`.
6. **Slow down near the goal:** within ~1.5 lookahead distances of the goal, scale
   the linear speed down proportionally so the robot eases in instead of
   overshooting.

## The ROS wrapper

**Subscribes:** **`/plan`** (`nav_msgs/Path`) — the path from A*. Cached in `path_`;
an empty path triggers a stop command.

**Publishes:** **`/cmd_vel`** (`geometry_msgs/msg/Twist`) — `linear.x = v`,
`angular.z = ω` — which `diff_drive_controller` (Lesson 3) turns into wheel speeds.

**Control loop:** a timer fires at `control_rate` (default **20 Hz**). Each tick
(`controlStep`):

1. If there's no path, **return without publishing** — this deliberately lets manual
   teleop own `/cmd_vel` when the robot isn't navigating (it won't fight you). When
   an empty `/plan` message arrives to clear a stale path, the subscriber publishes
   one stop command first, then the timer stays silent.
2. Look up the robot pose from TF (`map → base_footprint`), extract yaw from the
   quaternion (`yawFromQuaternion`, the Lesson 2 formula).
3. Convert the `Path` poses into plain `{x, y}` and call `computePursuitCommand()`.
4. If `goal_reached`: publish a stop, clear the path, log "goal reached." Otherwise
   publish the `Twist`.

Missing TF is handled with a throttled warning rather than a crash — robust to
startup ordering.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `lookahead_distance` | 0.3 m | how far ahead to aim — the key tuning knob |
| `desired_linear_speed` | 0.3 m/s | nominal forward speed |
| `max_angular_speed` | 1.5 rad/s | cap on turn rate |
| `goal_tolerance` | 0.2 m | distance to goal that counts as "arrived" |
| `control_rate` | 20 Hz | control loop frequency |
| `plan_topic` | `/plan` | path input |
| `cmd_vel_topic` | `/cmd_vel` | velocity-command output |

As a small ROS API repetition, the wrapper declares both topic names as string
parameters before creating its subscription and publisher. This separates the
controller's behavior from one hard-coded graph layout.

### Tuning intuition for `lookahead_distance`

- **Too short** → the robot reacts to every wiggle, oscillates, may overshoot turns.
- **Too long** → smooth but it cuts corners and tracks the path loosely.
  It's the classic stability-vs-accuracy tradeoff; 0.3 m suits this small robot.

## How it closes the loop

Tie the whole stack together:

```
particle_filter + EKF  →  map→base_footprint TF  (where am I)
a_star_planner         →  /plan                   (where to go)
pure_pursuit           →  /cmd_vel                (how to drive)
diff_drive_controller  →  wheel speeds            (Lesson 3)
Gazebo                 →  motion → /joint_states  (Lesson 6/3)
wheel_odometry → EKF → particle_filter            (updated "where am I")
```

That's the full sense → think → act loop running continuously. Pure pursuit is the
"act."

## Hands-on

First complete
[Lesson 11 implementation steps](implementation-steps/lesson-11-pure-pursuit.md).

Keep the Lesson 10 localization and planning processes running, then add the
controller in one new sourced terminal:

```sh
ros2 launch pure_pursuit pure_pursuit.launch.py
```

In another terminal, watch its output:

```sh
ros2 topic echo /cmd_vel       # watch the velocity commands while navigating
```

In RViz:

1. Set a goal with **2D Goal Pose** (Lesson 10) and watch the robot drive the
   `/plan` — `/cmd_vel` should show forward speed + turn rate that curves it along
   the path.
2. Watch it **ease to a stop** near the goal (the slowdown logic) and log "goal
   reached."
3. Tune `lookahead_distance` in `pure_pursuit.launch.py` and relaunch the
   controller. Try 0.6 vs. 0.15 and watch corner-cutting vs. oscillation.
4. With no goal set, drive with teleop — confirm pure pursuit stays quiet and
   doesn't fight you (the "no path → don't publish" behavior).

## Video supplement

- **LTC21 Tutorial Pure Pursuit** by Neuromorphic Workshop Telluride

## Check yourself

- What's the difference between a path (planning) and following it (control)?
- What is the lookahead point, and why search for it forward-only?
- Derive/explain the curvature `κ = 2·y_r / Ld²` and how `ω` follows from it.
- Why does the controller slow down near the goal, and why not publish when there's
  no path?
- How does `/cmd_vel` become actual wheel motion (trace it through earlier lessons)?

## Recap

**Pure pursuit** follows a path by steering toward a **lookahead point** a fixed
distance ahead and driving the **circular arc** to it, with curvature
`κ = 2·y_r / Ld²`. The core (`computePursuitCommand`) finds the lookahead point,
computes `(v, ω)`, slows near the goal, and reports goal-reached; the ROS wrapper
runs it at 20 Hz off the `map→base_footprint` TF and the `/plan`, publishing
`/cmd_vel`. This is the "act" that closes the full navigation loop.

➡️ **Next:** [Lesson 12 — Putting It All Together](lesson-12-bringup-full-stack.md).
