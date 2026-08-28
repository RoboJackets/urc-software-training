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
