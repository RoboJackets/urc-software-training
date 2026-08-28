# Terminal Commands

Run lesson commands from a terminal inside the TigerVNC desktop unless a lesson says
otherwise. If you prefer your own host terminal, use `docker compose exec
ros2-humble-vnc bash` to open an equivalent shell inside the running container.

## Start The Environment

From the repo root on the host:

```sh
./start.sh
```

Connect TigerVNC Viewer to `localhost:5901`, then open **Terminal Emulator** on the
VNC desktop. Host-terminal alternative:

```sh
docker compose exec ros2-humble-vnc bash
```

## Build And Source

Build:

```sh
colcon build --symlink-install
```

Source in every new VNC terminal or `docker compose exec` shell:

```sh
source install/setup.bash
```

Build one package:

```sh
colcon build --symlink-install --packages-select wheel_odometry
```

Use `--packages-select` only after that package exists in your starter repo or has
been created in its lesson.

Build through one package and its dependencies:

```sh
colcon build --symlink-install --packages-up-to a_star_planner
```

Likewise, `--packages-up-to` only works once the named package exists.

## Run The Main Stack

Simulator:

```sh
ros2 launch robonav_training_bringup sim.launch.py
```

Navigation stack:

```sh
ros2 launch robonav_training_bringup navigation.launch.py
```

The navigation stack is the Lesson 12 integration target. Before then, run only the
packages that already exist and have been implemented.

Teleop:

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Run Individual Packages

These commands are references for after the corresponding package exists or its
starter scaffold has been added. Some nodes need upstream topics/TF to do useful
work even if they can launch by themselves; for example `pure_pursuit` needs
`/plan` and `map -> base_footprint`.

```sh
ros2 launch ros_cpp_practice number_scaler.launch.py
ros2 launch wheel_odometry wheel_odometry.launch.py
ros2 launch map_server map_server.launch.py
ros2 launch particle_filter particle_filter.launch.py
ros2 launch a_star_planner a_star_planner.launch.py
ros2 launch pure_pursuit pure_pursuit.launch.py
ros2 launch ekf_localization ekf_localization.launch.py
```

Run a standalone component executable:

This specific command works after Lesson 5 registers the standalone executable.

```sh
ros2 run wheel_odometry wheel_odometry
```

Run with parameter overrides:

Use these after the relevant package exists and the executable is installed.

```sh
ros2 run wheel_odometry wheel_odometry --ros-args -p wheel_separation:=0.30
ros2 run pure_pursuit pure_pursuit --ros-args -p lookahead_distance:=0.45
```

## Inspect Packages And Interfaces

```sh
ros2 pkg list | grep robonav
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show nav_msgs/msg/OccupancyGrid
ros2 interface show sensor_msgs/msg/LaserScan
```

After Lesson 5 creates `wheel_odometry`, you can inspect its installed prefix:

```sh
ros2 pkg prefix wheel_odometry
```

## Inspect Nodes, Components, Topics

```sh
ros2 node list
ros2 component list
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic info /map --verbose
ros2 topic type /plan
```

Topic-specific commands work only after that topic exists.

Echo topics:

Only echo topics whose publisher is currently running. For example,
`/joint_states`, `/lidar/scan`, and `/imu/data` exist with the simulator; later
topics appear as you implement and launch each package.

```sh
ros2 topic echo /joint_states
ros2 topic echo /wheel/odometry
ros2 topic echo /odometry/filtered
ros2 topic echo /amcl_pose
ros2 topic echo /plan
ros2 topic echo /cmd_vel
```

Measure rates:

```sh
ros2 topic hz /joint_states
ros2 topic hz /lidar/scan
ros2 topic hz /imu/data
```

Publish a command manually:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Publish a goal manually:

```sh
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Inspect Parameters

These node names exist only after the relevant node is running.

```sh
ros2 param list
ros2 param list /a_star_planner
ros2 param get /a_star_planner inflation_radius
ros2 param get /pure_pursuit lookahead_distance
```

These training nodes read parameters at startup. To change behavior, relaunch with a
parameter override or edit the launch file used by the component container.

## Inspect TF

Print a live transform:

Each transform appears only after its owner is running: `base_footprint ->
lidar_link` with the simulator, `odom -> base_footprint` after the EKF, and
`map -> odom` after the particle filter.

```sh
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint lidar_link
```

Generate a frame diagram:

```sh
ros2 run tf2_tools view_frames
```

Expected main tree in the full stack:

```text
map -> odom -> base_footprint -> base_link -> sensor links
```

## Debugging Checklist

If a package is not found:

```sh
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep package_name
```

If a topic is missing:

```sh
ros2 node list
ros2 topic list
```

If a callback does not run:

```sh
ros2 topic info /topic_name --verbose
ros2 topic hz /topic_name
```

Check:

- Does the publisher exist?
- Does the subscriber exist?
- Do the message types match?
- Are the QoS settings compatible?

If TF lookup fails:

```sh
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo parent_frame child_frame
```

Check:

- Are both frame names spelled correctly?
- Is the node that owns the transform running?
- Is `use_sim_time` consistent?

If launch changes do not appear:

```sh
colcon build --symlink-install
source install/setup.bash
ros2 launch package_name launch_file.launch.py
```

Python launch files are symlinked with `--symlink-install`, but you still need to
source new terminals so ROS sees newly built packages.
