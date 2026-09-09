# Project 5 — Wheel Odometry

> **Goal:** Convert wheel angles from `/joint_states` into pose and velocity on
> `/wheel/odometry`.

Keep the [Project 5 reference](../reference/project-05-wheel-odometry.md) open.
It contains the incomplete C++ and launch scaffolds used below.

## 1. Create The Package

```sh
cd /workspace/src # Double check that you're in the right src directory (the one with a_star_planner, map_server, particle_filter, etc.) before running 'ros pkg create'
ros2 pkg create wheel_odometry --build-type ament_cmake --dependencies \
  rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs tf2 \
  tf2_geometry_msgs robonav_training_common
cd wheel_odometry
mkdir -p include/wheel_odometry launch
```

It's important to understand that the `urc-software-training/src/` file is where all of the packages go, everything outside of that src file is for the build, install, log, and other setup files. Most—if not all—of what you’ll be doing will be within the `urc-software-training/src/` file.

Create:

```text
include/wheel_odometry/wheel_odometry.hpp
src/wheel_odometry.cpp
launch/wheel_odometry.launch.py
```

Copy the complete [`package.xml`](../reference/project-05-wheel-odometry.md#packagexml)
from the reference.

## 2. Create A Buildable Node Skeleton

Use the [header and source scaffolds](../reference/project-05-wheel-odometry.md#node-scaffolds).
The class needs:

- parameters for wheel radius, wheel separation, and both joint names
- pose state `x_`, `y_`, `theta_`
- previous wheel positions, previous timestamp, and a first-message flag
- a `JointState` subscription and `Odometry` publisher
- private methods `onJointState(...)` and `publishOdometry(...)`

Copy the complete
[`CMakeLists.txt`](../reference/project-05-wheel-odometry.md#cmakeliststxt)
from the reference.

## 3. Set Up The Constructor

Use these defaults:

| Parameter | Default |
| --- | --- |
| `wheel_radius` | `0.075` m |
| `wheel_separation` | `0.34` m |
| `left_joint_name` | `left_wheel_joint` |
| `right_joint_name` | `right_wheel_joint` |

Publish `/wheel/odometry` and subscribe to `/joint_states`, both with queue depth
`10`. Use the [constructor shapes](../reference/project-05-wheel-odometry.md#constructor-shapes).

## 4. Implement `onJointState`

Start from the [method shapes](../reference/project-05-wheel-odometry.md#method-shapes),
then follow this order:

1. Find the left and right wheel indices by name. Use the provided
   `findJointIndex` helper from the reference; do not assume array order.
2. Return if either name is missing or either index is outside `msg->position`.
3. Read both wheel positions and convert `msg->header.stamp` to `rclcpp::Time`.
4. On the first valid message, save both positions and the timestamp, publish
   zero velocity, and return.
5. Calculate `dt`. Return if `dt <= 0`.
6. Calculate and integrate the odometry values below.
7. Publish, then save the current positions and timestamp for the next callback.

Use the math from Lesson 4:

```text
d_left   = (left - previous_left) * wheel_radius
d_right  = (right - previous_right) * wheel_radius
d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / wheel_separation

mid_theta = theta + d_theta / 2
x += d_center * cos(mid_theta)
y += d_center * sin(mid_theta)
theta = normalizeAngle(theta + d_theta)

vx = d_center / dt
wz = d_theta / dt
```

Use `robonav_training::normalizeAngle` for the final heading.

## 5. Implement `publishOdometry`

Create one `nav_msgs::msg::Odometry` and fill:

- `header.stamp` from the callback timestamp
- `header.frame_id = "odom"`
- `child_frame_id = "base_footprint"`
- x and y position
- orientation converted from `theta_`
- `twist.twist.linear.x = vx`
- `twist.twist.angular.z = wz`

Use the [odometry message lines](../reference/project-05-wheel-odometry.md#odometry-message-lines).
This node does not publish TF; Project 7's EKF owns `odom -> base_footprint`.

## 6. Add The Launch File

Complete the [launch scaffold](../reference/project-05-wheel-odometry.md#launch-scaffold).
It should run the generated `wheel_odometry` executable and pass `use_sim_time`.

## 7. Build And Test

```sh
cd /workspace
colcon build --symlink-install --packages-select wheel_odometry
source install/setup.bash
```

Run these in separate sourced terminals:

```sh
ros2 launch robonav_training_bringup sim.launch.py
ros2 launch wheel_odometry wheel_odometry.launch.py
ros2 topic echo /wheel/odometry
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Check only these results:

- forward driving increases x
- spinning changes yaw
- frames are `odom` and `base_footprint`
- no `odom -> base_footprint` TF exists yet

If the topic does not update, run `ros2 topic echo /joint_states --once` and
check the configured joint names first.
