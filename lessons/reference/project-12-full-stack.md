# Project 12 Reference — Full-Stack Integration

Use these incomplete pieces with the Project 12 instructions.

## Launch Scaffold

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")

    # TODO: create EKF include and component container

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("map_server"), "maps", "training_map.yaml"]
                ),
            ),
            # TODO: return EKF include and container
        ]
    )
```

## EKF Include

```python
ekf = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("ekf_localization"),
             "launch", "ekf_localization.launch.py"]
        )
    ),
    launch_arguments={"use_sim_time": use_sim_time}.items(),
)
```

## Component Shape

```python
ComposableNode(
    package="PACKAGE",
    plugin="PLUGIN",
    name="NODE_NAME",
    parameters=[
        {
            "use_sim_time": use_sim_time,
            # TODO: node-specific parameters
        }
    ],
)
```

Place the five components inside:

```python
container = ComposableNodeContainer(
    name="robonav_navigation_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    output="screen",
    composable_node_descriptions=[
        # TODO: five ComposableNode objects
    ],
)
```

Map-server parameter:

```python
parameters=[{"use_sim_time": use_sim_time, "map_yaml_path": map_yaml}],
```

Particle-filter parameters:

```python
parameters=[{
    "use_sim_time": use_sim_time,
    "odom_topic": "/odometry/filtered",
    "scan_topic": "/lidar/scan",
    "estimated_pose_topic": "/amcl_pose",
    "base_frame": "base_footprint",
    "lidar_frame": "lidar_link",
}],
```

## `package.xml`

Copy this complete file to `robonav_training_bringup/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robonav_training_bringup</name>
  <version>0.0.1</version>
  <description>Minimal ROS 2 Humble Gazebo Ignition and RViz bringup for a differential-drive training robot.</description>
  <maintainer email="joey.marra2007@gmail.com">URC Training</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>diff_drive_controller</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>robonav_training_description</exec_depend>
  <exec_depend>rclcpp_components</exec_depend>
  <exec_depend>wheel_odometry</exec_depend>
  <exec_depend>ekf_localization</exec_depend>
  <exec_depend>map_server</exec_depend>
  <exec_depend>particle_filter</exec_depend>
  <exec_depend>a_star_planner</exec_depend>
  <exec_depend>pure_pursuit</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## `CMakeLists.txt`

Copy this complete file to `robonav_training_bringup/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(robonav_training_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```
