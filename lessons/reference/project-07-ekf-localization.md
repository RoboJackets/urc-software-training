# Project 7 Reference — EKF Localization

Use these incomplete pieces with the Project 7 instructions.

## Config Scaffold

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.2
    two_d_mode: true
    publish_tf: true
    use_sim_time: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    odom0: /wheel/odometry
    odom0_config: [
      # TODO: 15 booleans
    ]
    odom0_differential: false
    odom0_relative: false

    imu0: /imu/data
    imu0_config: [
      # TODO: 15 booleans
    ]
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
```

Mask order:

```text
x, y, z, roll, pitch, yaw,
vx, vy, vz, vroll, vpitch, vyaw,
ax, ay, az
```

## Launch Scaffold

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = PathJoinSubstitution(
        [FindPackageShare("ekf_localization"), "config", "ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                # TODO: load config_file and pass use_sim_time
            ),
        ]
    )
```

Parameter shape:

```python
parameters=[config_file, {"use_sim_time": use_sim_time}],
```

## CMake Fragment

Add before `ament_package()`:

```cmake
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
```
