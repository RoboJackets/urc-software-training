# Project 11 — Pure Pursuit

> **Goal:** Complete the ROS-free function that turns a path into linear and
> angular velocity commands.

Keep the [Project 11 reference](../reference/project-11-pure-pursuit.md) open.

## 1. Parameterize The ROS Wrapper

In `src/pure_pursuit/src/pure_pursuit.cpp`, replace the two hard-coded topic
strings with declared string parameters:

| Parameter | Default |
| --- | --- |
| `plan_topic` | `/plan` |
| `cmd_vel_topic` | `/cmd_vel` |

Use the [wrapper parameter lines](../reference/project-11-pure-pursuit.md#wrapper-parameters)
in the existing subscription and publisher.

## 2. Complete The Controller Core

Edit:

```text
src/pure_pursuit/src/pure_pursuit_core.cpp
```

Add includes for `<algorithm>` and
`"robonav_training_common/angles.hpp"`.

The empty-path check, goal check, and nearest-point search are already complete.
Fill the three TODO blocks:

1. Starting at `nearest_index`, choose the first path point at least
   `lookahead_distance` from the robot. If none exists, use `path.back()`.
2. Translate that point relative to the robot, then rotate it by `-robot_yaw` to
   get robot-frame `x_r` (forward) and `y_r` (left).
3. If `x_r < 0`, command linear speed `0.05` and turn toward the point at
   `max_angular_speed`. Otherwise:
   - calculate `Ld = hypot(x_r, y_r)`
   - calculate curvature `2 * y_r / (Ld * Ld)`, guarded against zero
   - slow proportionally when the goal is closer than `1.5 * lookahead_distance`
   - set angular speed to `linear * curvature`, clamped to
     `[-max_angular_speed, max_angular_speed]`

Use the reference's [lookahead](../reference/project-11-pure-pursuit.md#lookahead-scaffold),
[transform](../reference/project-11-pure-pursuit.md#world-to-robot-transform), and
[command](../reference/project-11-pure-pursuit.md#command-shape) pieces. Return a
`PursuitCommand` using the field names declared in the header.

## 3. Build And Check

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select pure_pursuit
source install/setup.bash
```

Then follow the Lesson 11 hands-on checkpoint. Confirm:

- an empty path produces no movement
- a point on the left produces positive angular velocity
- a point on the right produces negative angular velocity
- the robot slows and stops near the goal

If turning signs are reversed, recheck the world-to-robot transform.
