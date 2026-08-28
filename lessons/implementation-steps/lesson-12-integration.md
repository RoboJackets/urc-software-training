# Project 12 — Full-Stack Integration

> **Goal:** Wire the completed localization, planning, and control packages into
> `navigation.launch.py`.

Keep the [Project 12 reference](../reference/project-12-full-stack.md) open.

## 1. Check The Inputs

Before editing the launch file, confirm the workspace builds and these earlier
projects are complete:

- Project 5: `/wheel/odometry`
- Project 7: `/odometry/filtered` and `odom -> base_footprint`
- Projects 9–11: their packages build successfully

Do not debug integration while an individual package still fails to build.

## 2. Update Bringup Dependencies

Add these to `src/robonav_training_bringup/package.xml`:

```xml
<exec_depend>wheel_odometry</exec_depend>
<exec_depend>ekf_localization</exec_depend>
```

The other navigation-package dependencies are already present.

## 3. Complete `navigation.launch.py`

Edit:

```text
src/robonav_training_bringup/launch/navigation.launch.py
```

Start from the [launch scaffold](../reference/project-12-full-stack.md#launch-scaffold)
and complete three pieces:

1. Declare `use_sim_time` and `map` launch arguments. The map default is
   `map_server/maps/training_map.yaml`, located with `FindPackageShare`.
2. Include `ekf_localization.launch.py` and forward `use_sim_time`.
3. Create one `ComposableNodeContainer` containing these five components:

| Package | Plugin | Node name |
| --- | --- | --- |
| `wheel_odometry` | `robonav_training::WheelOdometry` | `wheel_odometry` |
| `map_server` | `robonav_training::MapServer` | `map_server` |
| `particle_filter` | `particle_filter::ParticleFilter` | `particle_filter` |
| `a_star_planner` | `robonav_training::AStarPlanner` | `a_star_planner` |
| `pure_pursuit` | `robonav_training::PurePursuit` | `pure_pursuit` |

Pass `use_sim_time` to every node. Also pass:

- map server: `map_yaml_path = map`
- particle filter:
  - `odom_topic = /odometry/filtered`
  - `scan_topic = /lidar/scan`
  - `estimated_pose_topic = /amcl_pose`
  - `base_frame = base_footprint`
  - `lidar_frame = lidar_link`

Those particle-filter overrides are required because its standalone defaults do
not match this simulator. The reference shows the component parameter shape.

## 4. Build And Run

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run in separate sourced terminals:

```sh
# Terminal 1
ros2 launch robonav_training_bringup sim.launch.py

# Terminal 2
ros2 launch robonav_training_bringup navigation.launch.py

# Terminal 3, only while localizing
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Drive until the particle cloud converges, stop teleop, then send a **2D Goal
Pose** in RViz.

## 5. Check The Stack In Order

```sh
ros2 topic echo /wheel/odometry --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo --qos-reliability reliable \
  --qos-durability transient_local /map --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
```

Stop at the first missing item and fix its producer. A successful run has a
stable particle cloud, a visible plan, and a robot that follows the path and
stops near the goal.
