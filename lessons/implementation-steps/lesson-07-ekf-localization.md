# Project 7 — EKF Localization

> **Goal:** Configure `robot_localization` to combine wheel odometry and IMU data
> into `/odometry/filtered` and TF `odom -> base_footprint`.

You are not writing an EKF. Keep the
[Project 7 reference](../reference/project-07-ekf-localization.md) open for the
YAML and launch scaffolds.

## 1. Create The Package

```sh
cd /workspace/src
ros2 pkg create ekf_localization --build-type ament_cmake \
  --dependencies robot_localization
cd ekf_localization
mkdir -p config launch
```

Create:

```text
config/ekf.yaml
launch/ekf_localization.launch.py
```

Add these to `package.xml` before `<export>`:

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Keep the generated `robot_localization` dependency.

## 2. Update CMake

This package has no C++ target. Add the install block from the
[reference](../reference/project-07-ekf-localization.md#cmake-fragment)
before `ament_package()`.

## 3. Write `ekf.yaml`

Start from the [config scaffold](../reference/project-07-ekf-localization.md#config-scaffold).
Set:

- `frequency: 30.0`
- `sensor_timeout: 0.2`
- `two_d_mode: true`
- `publish_tf: true`
- frames: `map`, `odom`, `base_footprint`, with `world_frame: odom`
- `odom0: /wheel/odometry`
- `imu0: /imu/data`
- `imu0_remove_gravitational_acceleration: true`
- all `*_differential` and `*_relative` settings to `false`

Each sensor mask has 15 booleans in this order:

```text
x, y, z, roll, pitch, yaw,
vx, vy, vz, vroll, vpitch, vyaw,
ax, ay, az
```

Enable only:

- wheel odometry: `vx` and `vyaw`
- IMU: `yaw` and `vyaw`

Count each list before continuing; each must contain exactly 15 values.

## 4. Write The Launch File

Complete the [launch scaffold](../reference/project-07-ekf-localization.md#launch-scaffold).
It must:

- launch package `robot_localization`, executable `ekf_node`
- name the node `ekf_filter_node` so it matches the YAML key
- load the installed `config/ekf.yaml`
- pass the `use_sim_time` launch argument

## 5. Build And Test

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ekf_localization
source install/setup.bash
```

Run the simulator, wheel odometry, and EKF in separate sourced terminals:

```sh
ros2 launch robonav_training_bringup sim.launch.py
ros2 launch wheel_odometry wheel_odometry.launch.py
ros2 launch ekf_localization ekf_localization.launch.py
```

Check:

```sh
ros2 topic echo /odometry/filtered
ros2 run tf2_ros tf2_echo odom base_footprint
```

If there is no output, verify `/wheel/odometry` and `/imu/data` are publishing
before changing the YAML.
