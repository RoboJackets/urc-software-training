# Practice — Your First ROS 2 C++ Node

> **Goal:** Build one tiny node before wheel odometry so ROS syntax and robotics
> math do not arrive at the same time.

Do this after Lesson 1 and before Lesson 2. The `src/ros_cpp_practice` package is
intentionally absent at first; create and implement the small node yourself.

## Where This Lives In The Repo

- `src/ros_cpp_practice/src/number_scaler.cpp` — node and component registration
- `src/ros_cpp_practice/CMakeLists.txt`, `package.xml` — build and dependencies
- `src/ros_cpp_practice/launch/number_scaler.launch.py` — component container

## C++ You Need For This Course

This course assumes you can already write variables, functions, `if` statements,
loops, and simple classes. These ROS-specific C++ forms are worth recognizing:

| Form | Read it as |
| --- | --- |
| `Type name;` | a variable named `name` |
| `Type & value` | a reference; use the existing object without copying it |
| `const Type & value` | read an existing object without copying or changing it |
| `thing.method()` | call a method on an object |
| `pointer->method()` | call a method through a pointer |
| `Package::Type` | `Type` inside the `Package` namespace |
| `Container<Message>` | a template specialized for `Message` |
| `Message::SharedPtr` | a shared pointer to a message owned with ROS |
| `.hpp` / `.cpp` | declarations / implementations |

You do not need to memorize the long ROS types. Copy their shape from the
[C++ node patterns](reference/cpp-node-patterns.md), then change the message type,
topic, callback, and member name deliberately.

## What The Node Does

```text
/practice/input -> NumberScaler(scale parameter) -> /practice/output
```

It subscribes to `std_msgs/msg/Float64`, multiplies `data` by `scale`, and publishes
another `Float64`. This is intentionally ordinary: one class, one parameter, one
subscriber, one publisher, one callback, and component registration.

## Create And Read It In Layers

From `/workspace/src`, create the package if it is absent:

```sh
ros2 pkg create ros_cpp_practice --build-type ament_cmake \
  --dependencies rclcpp rclcpp_components std_msgs
```

Stay oriented after the command: it creates the package at
`/workspace/src/ros_cpp_practice`. All paths in the rest of this practice are
relative to that package, not to `/workspace/src`:

```sh
cd /workspace/src/ros_cpp_practice
mkdir -p launch
```

That command creates `package.xml` and `CMakeLists.txt`. Before adding your C++
target, the generated empty package should build:

```sh
cd /workspace
colcon build --packages-select ros_cpp_practice
```

In `package.xml`, keep the generated `ament_cmake` build-tool dependency and the
three generated `<depend>` entries. Because this package also installs a Python
launch file, add:

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Create `/workspace/src/ros_cpp_practice/src/number_scaler.cpp`. Build it in this order:

1. Includes bring `rclcpp`, component registration, `Float64`, and `std::bind` into
   the translation unit.
2. `NumberScaler : public rclcpp::Node` makes the class a node. Its constructor
   accepts `rclcpp::NodeOptions` so a component container can create it.
3. The constructor declares `scale`, then creates ROS interfaces.
4. `onNumber` is called for every input message and publishes one output.
5. Member variables keep the parameter and ROS interfaces alive.
6. `RCLCPP_COMPONENTS_REGISTER_NODE` makes the class discoverable as a component.

The callback connection is the densest line:

```cpp
input_sub_ = create_subscription<std_msgs::msg::Float64>(
  "/practice/input", 10,
  std::bind(&NumberScaler::onNumber, this, std::placeholders::_1));
```

Read it as: subscribe to `Float64` on this topic, keep ten recent messages, and
call `this` object's `onNumber` method with the incoming message.

In `CMakeLists.txt`, find the dependencies, build a shared component library,
attach its ROS dependencies, and register the plugin. The `EXECUTABLE
number_scaler` registration option also generates the convenient standalone
executable used below. `package.xml` declares the same dependencies for ROS
tooling and other packages.

After `src/number_scaler.cpp` exists and is syntactically complete, add this below
the generated `find_package(...)` lines and before `ament_package()`:

```cmake
add_library(number_scaler_component SHARED src/number_scaler.cpp)
ament_target_dependencies(number_scaler_component
  rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(number_scaler_component
  PLUGIN "robonav_training::NumberScaler"
  EXECUTABLE number_scaler)

install(TARGETS number_scaler_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

Also set C++17 near the top of the file and make sure these lines exist:

```cmake
set(CMAKE_CXX_STANDARD 17)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
```

Do not add `src/number_scaler.cpp` to CMake before the file exists: CMake treats a
listed-but-missing source as a configuration error, which would break the whole
workspace build.

Create `/workspace/src/ros_cpp_practice/launch/number_scaler.launch.py` by adapting
the complete **Composable Node Launch Pattern** in
[Launch and parameters](reference/launch-and-parameters.md). Use package
`ros_cpp_practice`, plugin `robonav_training::NumberScaler`, node name
`number_scaler`, and container name `practice_container`. Declare a `scale` launch
argument with default `2.0`, then pass `parameters=[{"scale": scale}]` to the
`ComposableNode`. The plugin string must exactly match both the C++ registration
macro and the CMake registration.

## Build And Run

From `/workspace`:

```sh
colcon build --symlink-install --packages-select ros_cpp_practice
source install/setup.bash
ros2 launch ros_cpp_practice number_scaler.launch.py
```

The launch command is the normal course path: it loads the component into a
container. Component registration also generates a convenient standalone
executable, so this works as an alternative:

```sh
ros2 run ros_cpp_practice number_scaler
```

Use either the launch command or `ros2 run`, not both at once.

In a second sourced terminal, watch the output:

```sh
ros2 topic echo /practice/output
```

In a third sourced terminal, publish one input:

```sh
ros2 topic pub --once /practice/input std_msgs/msg/Float64 "{data: 3.0}"
```

The output should be `6.0`. Restart the launch with `scale:=0.5`; the same input
should produce `1.5`. For the standalone form, the equivalent override is
`ros2 run ros_cpp_practice number_scaler --ros-args -p scale:=0.5`.

Inspect rather than guess:

```sh
ros2 node info /number_scaler
ros2 param get /number_scaler scale
ros2 topic info /practice/input --verbose
```

➡️ **Next:** [Lesson 2 — Coordinate Transforms & TF2](lesson-02-coordinate-transforms-and-tf.md).
