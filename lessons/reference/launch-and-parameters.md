# Launch And Parameters

Launch files start nodes with a repeatable configuration. In this repo, most
custom C++ nodes are launched as composable nodes inside a component container.

## Standalone Node Launch Pattern

Use this minimal scaffold for a normal executable process. Add launch arguments
and parameters later from their separate entries.

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
            ),
        ]
    )
```

Created in Lesson 7:

- `src/ekf_localization/launch/ekf_localization.launch.py`

## Composable Node Launch Pattern

Use this minimal scaffold for a C++ component registered with
`RCLCPP_COMPONENTS_REGISTER_NODE`. Add arguments and parameters only when needed.

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="example_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="example_package",
                        plugin="robonav_training::ExampleNode",
                        name="example_node",
                    ),
                ],
            ),
        ]
    )
```

Answer-key or later-starter examples:

- `src/wheel_odometry/launch/wheel_odometry.launch.py`
- `src/a_star_planner/launch/a_star_planner.launch.py`
- `src/robonav_training_bringup/launch/navigation.launch.py`

## Multiple Components In One Container

This is the pattern used by the navigation stack.
The full navigation stack is wired in Lesson 12, after the component packages exist.

```python
ComposableNodeContainer(
    name="robonav_navigation_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    output="screen",
    composable_node_descriptions=[
        ComposableNode(
            package="wheel_odometry",
            plugin="robonav_training::WheelOdometry",
            name="wheel_odometry",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        ComposableNode(
            package="a_star_planner",
            plugin="robonav_training::AStarPlanner",
            name="a_star_planner",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ],
)
```

Why use one container:

- Less process overhead.
- Components can be managed together.
- The training code stays close to how larger ROS systems are composed.

## Launch Arguments

Imports:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
```

Declare an argument:

```python
DeclareLaunchArgument("use_sim_time", default_value="true")
```

Read it:

```python
use_sim_time = LaunchConfiguration("use_sim_time")
```

Pass it to a node:

```python
parameters=[{"use_sim_time": use_sim_time}]
```

Override it from the terminal:

```sh
ros2 launch wheel_odometry wheel_odometry.launch.py use_sim_time:=false
```

This works after Lesson 5 creates `wheel_odometry`.

## Finding Package Files

Use `FindPackageShare` when a launch file needs a config, map, URDF, or RViz file.

```python
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

map_yaml = PathJoinSubstitution(
    [
        FindPackageShare("map_server"),
        "maps",
        "training_map.yaml",
    ]
)
```

Provided early or integration examples:

- `src/map_server/launch/map_server.launch.py`
- `src/robonav_training_bringup/launch/sim.launch.py`
- `src/robonav_training_bringup/launch/navigation.launch.py`

## Parameters In C++ And Launch

This example uses `pure_pursuit`, which appears after its starter scaffold is added.

C++:

```cpp
lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.3);
```

Launch:

```python
ComposableNode(
    package="pure_pursuit",
    plugin="robonav_training::PurePursuit",
    name="pure_pursuit",
    parameters=[
        {
            "use_sim_time": use_sim_time,
            "lookahead_distance": 0.35,
            "desired_linear_speed": 0.3,
        }
    ],
)
```

Inspect:

```sh
ros2 param list /pure_pursuit
ros2 param get /pure_pursuit lookahead_distance
```

Inspect parameters only after the node is running.

## Naming Rules

For this repo, prefer:

- Node class: `AStarPlanner`, `WheelOdometry`, `PurePursuit`
- Node runtime name: `a_star_planner`, `wheel_odometry`, `pure_pursuit`
- Package name: lowercase with underscores
- Plugin string: `robonav_training::ClassName`
- Topic names: absolute names like `/plan`, `/cmd_vel`, `/map`

Names for packages you create become available only after those lessons create or
scaffold the packages.

## Launch Debugging Commands

Use a package-specific launch command only after that package exists.

```sh
ros2 launch a_star_planner a_star_planner.launch.py
ros2 node list
ros2 component list
ros2 param list /a_star_planner
ros2 topic list
```

If a launch file cannot find a package, rebuild and source:

```sh
colcon build --symlink-install
source install/setup.bash
```
