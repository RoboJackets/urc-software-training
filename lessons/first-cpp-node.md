# Project 0 — Your First ROS 2 C++ Node

> **Goal:** Build `NumberScaler`, a small node that multiplies each input number
> by a parameter and publishes the result.

Complete this after Lesson 1. Keep the
[Project 0 reference](reference/project-00-number-scaler.md) open; it contains the
incomplete scaffolds used below.

```text
/practice/input -> NumberScaler(scale) -> /practice/output
```

## 1. Create The Package

In a TigerVNC terminal:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
cd src
ros2 pkg create ros_cpp_practice --build-type ament_cmake \
  --dependencies rclcpp rclcpp_components std_msgs
cd ros_cpp_practice
mkdir -p include/ros_cpp_practice launch
```

The package will contain:

```text
include/ros_cpp_practice/number_scaler.hpp
src/number_scaler.cpp
launch/number_scaler.launch.py
CMakeLists.txt
package.xml
```

## 2. Update `package.xml`

The creation command added the C++ dependencies. Add these launch dependencies
before `<export>`:

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

## 3. Write The Header

Create `include/ros_cpp_practice/number_scaler.hpp` from the
[header scaffold](reference/project-00-number-scaler.md#header-scaffold).

Add exactly these private items:

- `onNumber(std_msgs::msg::Float64::SharedPtr msg)`
- `double scale_`
- a `Float64` publisher shared pointer
- a `Float64` subscription shared pointer

The `.hpp` contains declarations only; function bodies go in the `.cpp`.

## 4. Write The Source

Create `src/number_scaler.cpp` from the
[source scaffold](reference/project-00-number-scaler.md#source-scaffold).

In the constructor:

1. Declare `scale` as a `double` parameter with default `2.0`.
2. Publish `Float64` messages on `/practice/output` with queue depth `10`.
3. Subscribe to `/practice/input` with queue depth `10` and bind `onNumber`.

In `onNumber`, use the [callback shape](reference/project-00-number-scaler.md#callback-shape)
and set the new message's `data` to `msg->data * scale_`.
Register the component as `robonav_training::NumberScaler` after the namespace.

## 5. Wire CMake

Add the [CMake fragments](reference/project-00-number-scaler.md#cmake-fragments)
before the generated `ament_package()` line. They must:

- build `src/number_scaler.cpp` as `number_scaler_component`
- expose `include/`
- attach `rclcpp`, `rclcpp_components`, and `std_msgs`
- register plugin `robonav_training::NumberScaler`
- generate executable `number_scaler`
- install the library, header, and launch directory

## 6. Write The Launch File

Create `launch/number_scaler.launch.py` from the
[launch scaffold](reference/project-00-number-scaler.md#launch-scaffold). It runs
the generated `number_scaler` executable with `scale` set to `2.0`.

## 7. Build And Test

Terminal 1:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ros_cpp_practice
source install/setup.bash
ros2 launch ros_cpp_practice number_scaler.launch.py
```

In every additional terminal, first run:

```sh
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then run one command:

```sh
# Terminal 2
ros2 topic echo /practice/output

# Terminal 3
ros2 topic pub --once /practice/input std_msgs/msg/Float64 "{data: 3.0}"
```

The output should be `6.0`. To test another value, stop the launch and run:

```sh
ros2 run ros_cpp_practice number_scaler --ros-args -p scale:=0.5
```

The same input should now produce `1.5`.

If the package is not found, rebuild and source `install/setup.bash`. If it does
not compile, fix the first compiler error before reading the rest.

➡️ **Next:** [Lesson 2 — Coordinate Transforms & TF2](lesson-02-coordinate-transforms-and-tf.md).
