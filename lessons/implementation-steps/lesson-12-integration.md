# Lesson 12 Implementation - Full Stack Integration

This is the checkpoint where all implemented packages become one system.

There are two launch files because they have different jobs:

- `sim.launch.py` creates the simulated robot and its sensors
- `navigation.launch.py` starts the nodes that estimate pose, plan, and drive

Keep them in separate terminals. This makes it possible to restart navigation
after a code change without restarting Gazebo.

## Before Wiring The Full Stack

Each piece should already pass every check that is possible before integration:

- simulator and teleop from Lesson 1
- wheel odometry topic from Lesson 5
- EKF odometry and `odom -> base_footprint` from Lesson 7
- map server `/map` from Lesson 8
- particle filter, A*, and pure-pursuit packages each build after their TODOs are
  completed

The particle filter, planner, and controller require the full topic/TF chain, so
their runtime checks are intentionally performed after you wire and launch this
file. If a package does not build alone, do not add it to
`navigation.launch.py` yet.

Also verify the topic and TF contracts between packages. A node can build and run
correctly by itself while still using a topic or frame name that the next node
does not expect:

```text
/joint_states -> wheel_odometry -> /wheel/odometry
              -> EKF            -> /odometry/filtered + odom -> base_footprint
/map + /lidar/scan + filtered odometry
              -> particle filter -> /amcl_pose + map -> odom
/map + /goal_pose + map-frame robot pose
              -> A*              -> /plan
/plan + map-frame robot pose
              -> pure pursuit    -> /cmd_vel
```

The complete TF chain should be `map -> odom -> base_footprint -> robot links`.
The particle filter owns the first dynamic edge and the EKF owns the second.
Never configure two nodes to publish the same TF edge.

## Integration Tasks

Students should inspect or complete the bringup wiring:

- `sim.launch.py` starts Gazebo, robot description, bridge, controllers, and RViz
- `navigation.launch.py` starts the EKF process
- `navigation.launch.py` starts a component container for project components
- the container loads wheel odometry, map server, particle filter, A*, and pure
  pursuit
- all nodes use `use_sim_time`
- launch parameters override tuning values used in the final demo

When adding `wheel_odometry` and `ekf_localization` to the full-stack launch, also
add them as `<exec_depend>` entries in
`src/robonav_training_bringup/package.xml`. They are intentionally omitted before
their packages exist so dependency resolution and the starter build remain clean.

Build `navigation.launch.py` in three parts:

1. Declare launch arguments for `use_sim_time` and the map YAML path. Locate
   package data with `FindPackageShare` and `PathJoinSubstitution`; source-tree
   paths will fail after installation or in another workspace.
2. Include `ekf_localization.launch.py` with `IncludeLaunchDescription`, forwarding
   `use_sim_time`. `robot_localization` is a normal executable and does not use
   the project's component plugin namespace.
3. Create one `ComposableNodeContainer` and add the five project components.
   Every `ComposableNode` needs its package, exact registered plugin/class name,
   node name, and parameters.

The components and their plugin names are:

| Package | Plugin |
| --- | --- |
| `wheel_odometry` | `robonav_training::WheelOdometry` |
| `map_server` | `robonav_training::MapServer` |
| `particle_filter` | `particle_filter::ParticleFilter` |
| `a_star_planner` | `robonav_training::AStarPlanner` |
| `pure_pursuit` | `robonav_training::PurePursuit` |

Plugin strings are case-sensitive and must match each package's
`RCLCPP_COMPONENTS_REGISTER_NODE` entry. A component failing to load usually
means the plugin name is wrong, the library was not installed, or the current
terminal has not sourced the latest build.

Pass `{"use_sim_time": use_sim_time}` to every node. In simulation, ROS timers,
message stamps, and TF all need to agree on `/clock`; one node using wall time can
make otherwise valid transforms appear too old or too new.

Apply the final demo overrides deliberately:

- particle filter input topics and `base_footprint`/`lidar_link` frame names must
  match the simulator and EKF
- particle count, beam stride, sensor mixture, and motion noise tune localization
- the map server receives the selected map YAML path
- planner inflation radius accounts for robot clearance
- controller lookahead trades smoother steering against tighter path tracking

Prefer a small helper that creates the components sharing the common parameters,
but write the particle filter explicitly if that makes its longer tuning block
easier to read.

For example, a helper for the simpler components can be:

```python
common = [{"use_sim_time": use_sim_time}]

def component(package, plugin, name, extra_params=None):
    return ComposableNode(
        package=package,
        plugin=f"robonav_training::{plugin}",
        name=name,
        parameters=common + (extra_params or []),
    )

wheel_odometry = component(
    "wheel_odometry", "WheelOdometry", "wheel_odometry"
)
a_star = component(
    "a_star_planner", "AStarPlanner", "a_star_planner",
    [{"inflation_radius": 0.55}],
)
```

The particle filter cannot use that exact `robonav_training::` prefix because its
registered plugin is `particle_filter::ParticleFilter`; create it explicitly:

```python
particle_filter = ComposableNode(
    package="particle_filter",
    plugin="particle_filter::ParticleFilter",
    name="particle_filter",
    parameters=common + [{
        "odom_topic": "/odometry/filtered",
        "scan_topic": "/lidar/scan",
        "map_topic": "/map",
        # TODO: frame names and scoring/motion parameters
    }],
)
```

Finally, put these descriptions in the container's
`composable_node_descriptions` list. Defining an object is not enough by itself;
the launch system only starts actions returned inside `LaunchDescription`.

## Build And Source The Entire Overlay

From `/workspace`:

```sh
colcon build --symlink-install
source install/setup.bash
```

Build in a terminal where the base ROS installation is already sourced. The
`source install/setup.bash` command affects only the current shell, so run it in
every new terminal used for launch or CLI inspection. After changing C++ or
CMake, rebuild; after changing an installed launch/config file, rebuild and
re-source as well.

## Run Procedure

```sh
colcon build --symlink-install
source install/setup.bash
ros2 launch robonav_training_bringup sim.launch.py
ros2 launch robonav_training_bringup navigation.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use RViz:

- drive until the particle cloud converges
- send a 2D Goal Pose
- verify `/plan` appears and the robot follows it

This training particle filter starts with global localization and does not
subscribe to RViz's **2D Pose Estimate** (`/initialpose`). Move the robot so lidar
measurements can narrow the particle cloud instead.

Wait for the map, scan, and TF tree before judging localization. Drive a varied
route with both translation and rotation; repeated lidar views from only one
position may leave several particle clusters plausible. After the cloud forms
one stable cluster around the robot, stop teleop before sending a goal so teleop
and pure pursuit are not competing publishers on `/cmd_vel`.

In RViz, this setup is required for the full-stack checkpoint (the simulator-only
layout starts in `base_link` on purpose):

1. Expand **Global Options** and change **Fixed Frame** from `base_link` to `map`.
2. Click **Add** at bottom-left, use **By topic**, and add Map (`/map`), PoseArray
   (`/particle_cloud`), LaserScan (`/lidar/scan`), and Path (`/plan`). Add a TF
   display from **By display type** if it is not already present.
3. Confirm the map and live scan align after localization converges. Then use the
   top-toolbar **2D Goal Pose** tool; click-drag in free map space to publish
   `/goal_pose`.

## Debugging Order

Debug from producers toward consumers. Do not start at the controller when the
localization TF chain is incomplete.

Check the stack in dependency order:

```sh
ros2 topic list
ros2 node list
ros2 topic echo /joint_states --once
ros2 topic echo /wheel/odometry --once
ros2 topic echo /odometry/filtered --once
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /map --once
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
```

Interpret the first failure in that sequence:

- no `/joint_states`: simulator, bridge, or wheel controller problem
- no `/wheel/odometry`: wheel odometry is not loaded or rejected joint names
- no `/odometry/filtered` or `odom -> base_footprint`: EKF inputs/config problem
- no `/map`: map server path, component, or QoS problem
- no `/amcl_pose` or `map -> odom`: particle filter lacks map, scan, odometry, or
  lidar TF
- no `map -> base_footprint`: one of the two localization TF edges is missing
- no `/plan` after a goal: planner lacks map/TF/goal or endpoints are blocked
- path exists but no `/cmd_vel`: controller lacks the path or map-to-base TF

If A* logs `Start cell is blocked`, the localized robot is within the planner's
inflation radius of a wall or obstacle. Use teleop to move into visibly open map
space, let localization settle, and send the goal again; do not reduce inflation
just to hide a bad starting position.

Useful process and configuration checks are:

```sh
ros2 component list
ros2 component types
ros2 topic info /cmd_vel --verbose
ros2 param get /ekf_filter_node use_sim_time
ros2 param get /particle_filter use_sim_time
ros2 param get /a_star_planner use_sim_time
ros2 param get /pure_pursuit use_sim_time
```

`ros2 component list` confirms which nodes actually loaded into the shared
container. The verbose topic check reveals every `/cmd_vel` publisher; during
autonomous testing there should not be a running teleop command competing with
the controller.

## Full-Stack Acceptance Check

A completed run should demonstrate all of the following, in order:

1. Both launch commands stay running without component-load errors.
2. The occupancy map and live lidar align in RViz once localization converges.
3. `tf2_echo map base_footprint` updates continuously while driving.
4. A free-space goal produces a non-empty map-frame `/plan`.
5. `/cmd_vel` contains forward and turning commands and the robot follows the
   visible path.
6. The controller slows near the final pose, publishes a stop, and does not keep
   commanding motion after reaching the goal.

Save the earliest warning or error from the two launch terminals when a check
fails. Later errors are often just consequences of that first missing component,
topic, or transform.
