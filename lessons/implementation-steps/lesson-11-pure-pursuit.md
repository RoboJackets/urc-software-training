# Lesson 11 Implementation - Pure Pursuit Core

Students implement the pure path-following function. The package, message
subscriptions, TF lookup, timer, and `/cmd_vel` publisher can be provided as
starter code.

As in Lesson 10, the ROS-free core is separated from its wrapper. The wrapper
receives `/plan`, looks up the robot pose from TF, calls the core at a fixed rate,
and publishes the returned linear and angular velocity on `/cmd_vel`.

## Keep These References Open

- [2D transforms, clamping, and safe math](../reference/cpp-algorithm-patterns.md)
- [publisher, subscriber, parameter, timer, and TF patterns](../reference/cpp-node-patterns.md)
- [`Path` and `Twist` message fields](../reference/messages-and-qos.md)
- [package-specific build and source commands](../reference/terminal-commands.md#build-and-source)

Adapt the relevant geometry and node patterns to the provided controller types;
the reference is not a complete controller solution.

## Target Files

Fill in:

```text
src/pure_pursuit/src/pure_pursuit_core.cpp
```

Use the declarations in:

```text
src/pure_pursuit/include/pure_pursuit/pure_pursuit_core.hpp
```

Starter code should provide:

- `RobotPose`
- path point type
- `PursuitCommand`
- function signature for `computePursuitCommand(...)`
- parameters: lookahead distance, desired speed, max angular speed, goal tolerance

The starter core already handles an empty path, checks goal tolerance, and finds
the nearest path point. Complete the three numbered TODO blocks for lookahead
selection, the world-to-robot transform, and command generation. Until then it
returns a stop command, so unfinished controller math cannot move the robot or
break the workspace build.

Before the controller math, complete a small ROS wrapper checkpoint in
`pure_pursuit.cpp`: declare `plan_topic` and `cmd_vel_topic` string parameters with
defaults `/plan` and `/cmd_vel`, then pass them to the existing subscription and
publisher creation calls. Build once before editing the core so callback wiring
and geometry mistakes do not arrive together.

Read the `PursuitCommand` definition before editing. Its `linear` and `angular`
fields become `Twist.linear.x` and `Twist.angular.z`; `goal_reached` tells the
wrapper to publish a final stop and clear the stored path. All path points and the
robot pose passed into the core are in the `map` frame.

## Implement The Controller

Students write:

1. empty path handling: return stop
2. final-goal distance check: stop and set `goal_reached`
3. nearest path point search
4. lookahead point search forward from nearest point
5. fallback to the final point near the end of the path
6. transform lookahead point into the robot frame
7. curvature calculation: `kappa = 2 * y_r / Ld^2`
8. angular velocity: `omega = v * kappa`
9. clamp angular velocity to `max_angular_speed`
10. slow down near the goal
11. behind-robot handling, using heading error instead of the normal curvature formula

Implement the controller as the following pipeline:

1. Return `{0, 0, false}` for an empty path. There is no active goal to declare
   reached.
2. Measure from the robot to `path.back()`. Inside `goal_tolerance`, return
   `{0, 0, true}` immediately.
3. Find the nearest point by comparing squared Euclidean distance. Saving the
   nearest index prevents the lookahead search from pulling the robot backward
   along already-followed path points.
4. Starting at that index, choose the first point at least `lookahead_distance`
   from the robot. If none qualifies, select the final path point.
5. Translate the chosen point by subtracting the robot position, then rotate by
   negative robot yaw:

```text
x_r =  cos(yaw) * dx + sin(yaw) * dy   # forward
y_r = -sin(yaw) * dx + cos(yaw) * dy   # left
```

6. For a point in front of the robot, use the actual distance
   `Ld = hypot(x_r, y_r)`, compute curvature, and set `omega = linear * kappa`.
   Guard the division when `Ld` is almost zero and clamp omega symmetrically.
7. Reduce linear speed proportionally inside a small slow zone near the goal so
   the robot does not overshoot.
8. If `x_r < 0`, the normal arc is poorly behaved. Compute the wrapped heading
   error with `atan2(y_r, x_r)` and command a small forward creep plus a turn in
   the error's direction at no more than `max_angular_speed`.

ROS uses the conventional planar signs: positive x is forward, positive y is
left, and positive yaw/angular z turns counterclockwise. Thus a lookahead point
to the robot's left should produce positive curvature and angular velocity.

## Build And Check The Inputs

From `/workspace`:

```sh
colcon build --symlink-install --packages-select pure_pursuit
source install/setup.bash
```

The controller cannot produce a meaningful command until both a path and robot
pose exist:

```sh
ros2 topic echo /plan --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /cmd_vel
```

If `/plan` exists but `/cmd_vel` does not, inspect the controller logs for TF
lookup failures. If angular velocity has the wrong sign, recheck the world-to-
robot rotation and the meaning of positive `y_r`.

The main geometry maps directly into C++:

```cpp
const double dx = lookahead_x - robot_x;
const double dy = lookahead_y - robot_y;
const double x_r =  std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
const double y_r = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

const double Ld = std::hypot(x_r, y_r);
const double curvature =
  (Ld > 1e-6) ? (2.0 * y_r) / (Ld * Ld) : 0.0;

PursuitCommand command{};
command.linear = desired_linear_speed;
command.angular = std::clamp(
  command.linear * curvature, -max_angular_speed, max_angular_speed);
```

This snippet covers only the normal, point-in-front case. Perform empty-path,
goal, slowdown, and `x_r < 0` checks around it before returning the command.

Use the Lesson 2 angle wrapping helper where heading error is involved.

## Acceptance Checks

After Lesson 12 wiring:

```sh
ros2 topic echo /cmd_vel
```

Expected behavior:

- with no path, the controller does not fight teleop
- with a valid path, `/cmd_vel.linear.x` is positive
- turns appear in `/cmd_vel.angular.z`
- the robot slows near the goal
- goal reached publishes a stop and clears the path

The no-path behavior is intentionally different from publishing zero continuously:
the node stays silent so teleop can use `/cmd_vel`. Once a path is active, TF
failure should publish a stop for safety. Large oscillations usually call for a
larger lookahead or lower speed; wide corner cutting usually calls for a smaller
lookahead, within the limits of the robot and map.
