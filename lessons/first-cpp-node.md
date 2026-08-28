# Practice — Your First ROS 2 C++ Node

> **Goal:** Build one tiny node before wheel odometry so ROS syntax and robotics
> math do not arrive at the same time.

Do this after Lesson 1 and before Lesson 2. The `src/ros_cpp_practice` package is
intentionally absent at first; you will create it and implement the node. Work
through one numbered step at a time and build at the checkpoints.

## Where This Lives In The Repo

- `src/ros_cpp_practice/include/ros_cpp_practice/number_scaler.hpp` — class
  declaration and member variables
- `src/ros_cpp_practice/src/number_scaler.cpp` — constructor, callback, and
  component registration
- `src/ros_cpp_practice/CMakeLists.txt`, `package.xml` — build and dependencies
- `src/ros_cpp_practice/launch/number_scaler.launch.py` — component container

## C++ To Brush Up On

This course assumes you can write variables, functions, `if` statements, loops,
and simple classes. It also uses modern C++, especially references, `const`,
templates, and smart pointers. Review `std::unique_ptr`, `std::shared_ptr`,
`std::make_unique`, and `std::make_shared`; ROS types such as
`Message::SharedPtr` are shared-pointer aliases.

| Form | Read it as |
| --- | --- |
| `const Type & value` | read an existing object without copying or changing it |
| `pointer->method()` | call a method through a smart pointer |
| `Package::Type` | `Type` inside the `Package` namespace |
| `Container<Message>` | a template specialized for `Message` |
| `Message::SharedPtr` | a shared pointer to a ROS message |
| `.hpp` / `.cpp` | declarations / implementations |

Keep [C++ node patterns](reference/cpp-node-patterns.md) open. Do not copy that
page directly. Use the header, source, publisher, subscriber, and parameter parts
that fit NumberScaler, then use the names and behavior specified below.

## What The Node Does

```text
/practice/input -> NumberScaler(scale parameter) -> /practice/output
```

It receives a `std_msgs/msg/Float64`, multiplies its `data` field by `scale`, and
publishes another `Float64`. You are building one class with one parameter, one
subscriber, one publisher, and one callback.

## Step 1 — Open A Ready Terminal

Start the container and open **Terminal Emulator** inside TigerVNC as shown in
Lesson 1. Then run:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
```

The first command puts you at the workspace root. The second makes the installed
ROS 2 commands and packages available in this terminal.

## Step 2 — Create The Package And Folders

Run these commands from `/workspace/src`:

```sh
cd /workspace/src
ros2 pkg create ros_cpp_practice --build-type ament_cmake \
  --dependencies rclcpp rclcpp_components std_msgs
cd ros_cpp_practice
mkdir -p include/ros_cpp_practice launch
```

Check the result with `find . -maxdepth 3 -type f`. You should have
`CMakeLists.txt`, `package.xml`, and a `src/` directory. You just added the
`include/ros_cpp_practice/` and `launch/` directories.

Before adding C++ code, confirm the generated package builds:

```sh
cd /workspace
colcon build --packages-select ros_cpp_practice
source install/setup.bash
```

If this fails, fix it before continuing. For help, use the
[build and source reference](reference/terminal-commands.md#build-and-source).

## Step 3 — Update `package.xml`

Open `/workspace/src/ros_cpp_practice/package.xml`. Keep the generated
`ament_cmake` build-tool dependency and the generated `rclcpp`,
`rclcpp_components`, and `std_msgs` dependencies. Add these two runtime
dependencies before `<export>`:

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Use the [`package.xml` reference](reference/cmake-package.md#packagexml-dependency-syntax)
if you are unsure where dependencies belong.

## Step 4 — Write The Header

Create
`/workspace/src/ros_cpp_practice/include/ros_cpp_practice/number_scaler.hpp`.
Adapt the [header file skeleton](reference/cpp-node-patterns.md#header-file-skeleton)
for this application. Your header should contain, in this order:

1. `#pragma once`.
2. Includes for `rclcpp` and `std_msgs/msg/float64.hpp`.
3. The `robonav_training` namespace.
4. A `NumberScaler` class that publicly inherits from `rclcpp::Node`.
5. A public constructor taking `const rclcpp::NodeOptions & options`.
6. A private callback named `onNumber` taking
   `std_msgs::msg::Float64::SharedPtr msg`.
7. Private members for `double scale_`, a `Float64` publisher shared pointer, and
   a `Float64` subscription shared pointer.

The header says what the class contains. It should not contain the constructor or
callback bodies.

## Step 5 — Write The Source

Create `/workspace/src/ros_cpp_practice/src/number_scaler.cpp`. Adapt the
[source file skeleton](reference/cpp-node-patterns.md#source-file-skeleton), the
[parameter pattern](reference/cpp-node-patterns.md#parameter-pattern), the
[publisher pattern](reference/cpp-node-patterns.md#publisher-pattern), and the
[subscriber pattern](reference/cpp-node-patterns.md#subscriber-pattern).
Implement these pieces in order:

1. Include your own header as `"ros_cpp_practice/number_scaler.hpp"`, plus
   `<functional>` and the component-registration header.
2. Open the `robonav_training` namespace.
3. Define the constructor. Initialize the base node with the name
   `number_scaler` and the supplied `options`.
4. Declare a `double` parameter named `scale` with default `2.0` and store it in
   `scale_`.
5. Create a `Float64` publisher for `/practice/output` with queue depth `10`.
6. Create a `Float64` subscription for `/practice/input` with queue depth `10`.
   Bind it to `NumberScaler::onNumber`.
7. Define `onNumber`. Create an output message, set its `data` to
   `msg->data * scale_`, and publish it.
8. Close the namespace and register
   `robonav_training::NumberScaler` with
   `RCLCPP_COMPONENTS_REGISTER_NODE`.

The callback connection is the densest part. Read it as: subscribe to this
message type and topic, keep ten recent messages, and call this object's
`onNumber` method for each message.

## Step 6 — Wire CMake

Open `CMakeLists.txt`. Start with the
[CMake package skeleton](reference/cmake-package.md#cmake-package-skeleton), then
use [Adding a Component Library](reference/cmake-package.md#adding-a-component-library),
[Registering a Component](reference/cmake-package.md#registering-a-component),
and [Installing Launch, Config, Maps, URDF](reference/cmake-package.md#installing-launch-config-maps-urdf).
Make these changes:

1. Set C++17 and find `rclcpp`, `rclcpp_components`, and `std_msgs`.
2. Create a shared library named `number_scaler_component` from
   `src/number_scaler.cpp`.
3. Add this package's `include/` directory to that target.
4. Attach the three ROS dependencies.
5. Register plugin `robonav_training::NumberScaler` and generate executable
   `number_scaler`.
6. Install the library, the `include/` directory, and the `launch/` directory.

Do not wire a source file before it exists. A missing source makes CMake fail
before compilation begins.

## Step 7 — Write The Launch File

Create `launch/number_scaler.launch.py` by adapting the
[Composable Node Launch Pattern](reference/launch-and-parameters.md#composable-node-launch-pattern),
then add the `scale` argument using [Launch Arguments](reference/launch-and-parameters.md#launch-arguments)
and [Parameters in C++ and Launch](reference/launch-and-parameters.md#parameters-in-c-and-launch).
Use:

- package: `ros_cpp_practice`
- plugin: `robonav_training::NumberScaler`
- node name: `number_scaler`
- container name: `practice_container`
- launch argument: `scale`, default `2.0`
- component parameters: `[{"scale": scale}]`

The plugin string must exactly match the C++ registration macro and CMake.

## Step 8 — Build And Run

In the first terminal:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros_cpp_practice
source install/setup.bash
ros2 launch ros_cpp_practice number_scaler.launch.py
```

Fix the first compiler error first; later errors are often side effects. The
launch command loads the component into a container. As an alternative, run the
generated standalone executable with `ros2 run ros_cpp_practice number_scaler`.
Use one form at a time.

Open a second terminal and watch the output:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /practice/output
```

Open a third terminal and send one input:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /practice/input std_msgs/msg/Float64 "{data: 3.0}"
```

The output should be `6.0`. Stop the launch with `Ctrl-C`, then relaunch with:

```sh
ros2 launch ros_cpp_practice number_scaler.launch.py scale:=0.5
```

Send `3.0` again; the output should now be `1.5`.

## Step 9 — Inspect What You Built

With the node running, use the
[terminal reference](reference/terminal-commands.md#inspect-nodes-components-topics)
to inspect instead of guessing:

```sh
ros2 node info /number_scaler
ros2 param get /number_scaler scale
ros2 topic info /practice/input --verbose
```

Before moving on, confirm you can explain which declarations belong in the
`.hpp`, which implementations belong in the `.cpp`, why the ROS interfaces are
shared pointers, and why every new terminal must source its environment.

➡️ **Next:** [Lesson 2 — Coordinate Transforms & TF2](lesson-02-coordinate-transforms-and-tf.md).
