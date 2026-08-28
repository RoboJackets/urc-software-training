# Lesson 7 Implementation - EKF Package From Scratch

Build `ekf_localization` as a config-only package. Students are not writing a
Kalman filter implementation; they are configuring and launching
`robot_localization`.

The EKF combines two noisy measurements into one smooth local estimate:

```text
/wheel/odometry -- forward speed + turn rate --\
                                                  -> EKF -> /odometry/filtered
/imu/data ------- yaw + turn rate ---------------/       -> odom -> base_footprint
```

This lesson is mostly YAML and Python launch code, but those files are still a
ROS package so they can be installed and found with `ros2 launch`.

## Keep These References Open

- [config-only package and install patterns](../reference/cmake-package.md)
- [regular node launch pattern and parameters](../reference/launch-and-parameters.md)
- [message fields](../reference/messages-and-qos.md)
- [build, source, topic, and TF commands](../reference/terminal-commands.md)

Adapt only the YAML, launch, package, and terminal patterns needed for this EKF;
do not copy an unrelated reference example unchanged.

## Create The Package

From `/workspace/src`:

```sh
ros2 pkg create ekf_localization --build-type ament_cmake --dependencies \
  robot_localization
```

Create:

```text
src/ekf_localization/config/ekf.yaml
src/ekf_localization/launch/ekf_localization.launch.py
```

Wire only installs into `CMakeLists.txt`: install `config/` and `launch/`. There is
no C++ target in this package.

Keep the generated `package.xml` dependency on `robot_localization`. In
`CMakeLists.txt`, `find_package(ament_cmake REQUIRED)`, install both directories
under `share/${PROJECT_NAME}`, and finish with `ament_package()`. Without the
install rule, the files may exist in `src/` but `ros2 launch` will not find them.

Because this package contains no C++, its complete `CMakeLists.txt` is short:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Verify `package.xml` contains these dependencies inside `<package>...</package>`:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<exec_depend>robot_localization</exec_depend>
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

`ros2 pkg create` may generate `<depend>robot_localization</depend>` instead of
`<exec_depend>`; either works, but `exec_depend` communicates that this package
launches the installed EKF rather than compiling against it. Add the launch
dependencies manually because they were not part of the creation command.

## Write `ekf.yaml`

Configure:

- `frequency: 30.0`
- `sensor_timeout: 0.2`
- `two_d_mode: true`
- `publish_tf: true`
- `map_frame: map`
- `odom_frame: odom`
- `base_link_frame: base_footprint`
- `world_frame: odom`
- `use_sim_time: true`

Inputs:

- `odom0: /wheel/odometry`
- fuse wheel odometry `vx` and `vyaw` only
- do not fuse wheel odometry x, y, or yaw
- `imu0: /imu/data`
- fuse IMU yaw and yaw rate
- do not fuse linear acceleration
- set `imu0_remove_gravitational_acceleration: true`

Students should write the 15-element masks themselves after reading the lesson.

YAML parameters must be nested under the node name and `ros__parameters`:

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    # remaining parameters are indented to this level
```

Each sensor mask always uses this field order:

```text
[x, y, z, roll, pitch, yaw,
 vx, vy, vz, vroll, vpitch, vyaw,
 ax, ay, az]
```

For `odom0_config`, count carefully and set only index 6 (`vx`) and index 11
(`vyaw`) to `true`. For `imu0_config`, set only index 5 (`yaw`) and index 11
(`vyaw`) to `true`. Keep the masks split over rows matching the groups above; it
makes off-by-one errors much easier to spot.

The completed masks should look like this. Keeping each group on its own row makes
it possible to compare the booleans directly with the field-order diagram:

```yaml
odom0: /wheel/odometry
odom0_config: [false, false, false,
               false, false, false,
               true,  false, false,
               false, false, true,
               false, false, false]

imu0: /imu/data
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,
              false, false, false]
```

After filling a mask, count the values again. YAML accepts a list of the wrong
length, but `robot_localization` cannot interpret it as intended.

Why fuse these fields:

- wheel odometry measures body motion well over short periods, but its integrated
  pose drifts, so use its velocities rather than its x/y/yaw pose
- the IMU supplies heading and rotational velocity, giving the EKF another view
  of turning
- `two_d_mode` constrains z, roll, and pitch because this rover navigates on a
  plane
- `world_frame: odom` makes this a continuous local estimate; the particle
  filter later supplies the global `map -> odom` correction

Set `odom0_differential`, `odom0_relative`, `imu0_differential`, and
`imu0_relative` to `false`. Those switches change how measurements are
interpreted; they are not substitutes for selecting velocity fields in a mask.

## Write The Launch File

Launch `robot_localization`'s `ekf_node`:

- package: `robot_localization`
- executable: `ekf_node`
- node name: `ekf_filter_node`
- load `config/ekf.yaml`
- output to screen

Use `FindPackageShare("ekf_localization")` plus `PathJoinSubstitution` to locate
the installed YAML instead of hard-coding `/workspace/src/...`. Declare a
`use_sim_time` launch argument and pass parameters in this order:

```text
[path_to_ekf_yaml, {"use_sim_time": use_sim_time}]
```

The later dictionary overrides the YAML value when the caller supplies a launch
argument. The launch node name must stay `ekf_filter_node`, because the top-level
key in the YAML targets that name.

The core launch action can be scaffolded as:

```python
params = PathJoinSubstitution(
    [FindPackageShare("ekf_localization"), "config", "ekf.yaml"]
)

ekf = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node",
    output="screen",
    parameters=[params, {"use_sim_time": use_sim_time}],
)
```

Return this action inside a `LaunchDescription` together with the
`DeclareLaunchArgument` action.

## Build And Inspect The Configuration

From `/workspace`:

```sh
colcon build --symlink-install --packages-select ekf_localization
source install/setup.bash
ros2 launch ekf_localization ekf_localization.launch.py --show-args
```

Once the node is running, these commands help confirm that ROS loaded the file
you wrote rather than only using defaults:

```sh
ros2 param get /ekf_filter_node frequency
ros2 param get /ekf_filter_node base_link_frame
ros2 param get /ekf_filter_node odom0
ros2 param get /ekf_filter_node imu0
```

## Acceptance Checks

Run simulator, wheel odometry, then EKF:

```sh
ros2 launch robonav_training_bringup sim.launch.py
ros2 launch wheel_odometry wheel_odometry.launch.py
ros2 launch ekf_localization ekf_localization.launch.py
ros2 topic echo /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

Expected behavior:

- `/odometry/filtered` exists
- `odom -> base_footprint` exists
- the transform changes when driving
- there is still no `map -> odom`; that comes from the particle filter

If `/odometry/filtered` is absent, check that both input topics are publishing
and that all terminals use simulation time. If the topic exists but TF does not,
check `publish_tf`, the three frame names, and whether another node is already
publishing `odom -> base_footprint`. A TF edge should have exactly one owner.
