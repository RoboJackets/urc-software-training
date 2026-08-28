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

## `package.xml`

Copy this complete file to `ekf_localization/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ekf_localization</name>
  <version>0.1.0</version>
  <description>robot_localization EKF configuration (wheel odometry + IMU fusion) for the RoboNav software training.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Software Training</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_localization</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## `CMakeLists.txt`

Copy this complete file to `ekf_localization/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

find_package(ament_cmake REQUIRED)

# This package contributes no compiled code: it configures and launches the EKF
# from the robot_localization package. We only install the config and launch.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```
