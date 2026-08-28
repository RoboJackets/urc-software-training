# Lesson 0 — ROS 2 Foundations

> **Goal:** Understand what ROS 2 is and the handful of concepts —
> nodes, topics, messages, services, parameters, components, and the build
> system — that every other lesson depends on.

## Where this lives in the repo

ROS 2 is the framework the *whole* repo is built on. Concretely, look at:

- `src/*/package.xml` and `src/*/CMakeLists.txt` — every folder in `src/` is a
  ROS 2 **package**.
- `src/*/launch/*.launch.py` — **launch files** that start nodes.
- `src/map_server/src/map_server.cpp` — a small, complete provided **node** you can
  inspect now.
- `src/wheel_odometry/...` — the main running example below; this package is
  intentionally absent until you create it in Lesson 5.

## What is ROS 2?

ROS ("Robot Operating System") is **not** an operating system. It's a set of
libraries and tools for writing robot software as many small programs that talk to
each other over a network. The "2" is the modern rewrite (we use the **Humble**
release on Ubuntu 22.04).

Why split a robot's brain into many small programs instead of one big one?

- **Separation of responsibility.** One program reads wheel encoders, another
  fuses sensors, another plans paths. Each is small and testable.
- **Reusability.** The EKF in this repo is a standard package (`robot_localization`)
  that thousands of robots reuse.
- **Inspectability.** Because programs talk over named channels, you can tap any
  channel from the command line to see what's happening.

The pieces below are the vocabulary for that.

## 1. Nodes

A **node** is one running program that does one job. In this repo:

- `wheel_odometry` — turns wheel rotations into a position estimate.
- `particle_filter` — figures out where the robot is on the map.
- `a_star_planner` — plans a path.
- `pure_pursuit` — drives the path.

A node in C++ is a class that inherits from `rclcpp::Node`. After Lesson 5 creates
`src/wheel_odometry/include/wheel_odometry/wheel_odometry.hpp`, you'll see:

```cpp
class WheelOdometry : public rclcpp::Node { ... };
```

That single line means "this is a ROS 2 node." `rclcpp` is the C++ ROS client
library. ROS also has a Python client library called `rclpy`, but this repo's nodes
are C++. Its Python launch files use the separate `launch` and `launch_ros` APIs.

## 2. Topics, messages, and publish/subscribe

Nodes mostly talk through **topics**. A topic is a named channel (e.g. `/cmd_vel`)
that carries one **message type** (e.g. `geometry_msgs/Twist`, a velocity).

- A node that sends data **publishes** to a topic.
- A node that receives data **subscribes** to a topic.

This is *anonymous many-to-many*: a publisher doesn't know or care who is
listening. `pure_pursuit` publishes `/cmd_vel`; it has no idea whether the real
robot, the simulator, or a logger is reading it. That decoupling is the whole
point.

> The live set of all running nodes and the topics connecting them is called the
> **ROS graph**. When a command below "asks the ROS graph," it's querying this
> running network — which is why nothing has to be configured in advance to inspect
> it.

Concrete example from the `wheel_odometry.cpp` you create in Lesson 5:

- It **subscribes** to `/joint_states` (type `sensor_msgs/JointState`) — the wheel
  encoder readings.
- It **publishes** `/wheel/odometry` (type `nav_msgs/Odometry`) — its estimate of
  where the robot is.

A **message** is just a typed struct. `nav_msgs/Odometry` contains a header
(timestamp + frame), a pose (position + orientation), and a twist (velocity).
You'll meet a handful of message types over and over:

| Message | Carries | Used in |
| --- | --- | --- |
| `geometry_msgs/Twist` | linear + angular velocity | `/cmd_vel` |
| `nav_msgs/Odometry` | pose + velocity estimate | odometry, EKF |
| `sensor_msgs/LaserScan` | a ring of lidar ranges | particle filter |
| `sensor_msgs/Imu` | acceleration + angular rate | EKF |
| `nav_msgs/OccupancyGrid` | the map as a grid | map server, planner |
| `nav_msgs/Path` | a list of waypoints | planner → controller |

### Inspect topics from the TigerVNC terminal

After starting the simulator in Lesson 1, these are your bread and butter:

```sh
ros2 topic list                 # every topic currently alive
ros2 topic echo /joint_states --once  # print one wheel-state message
ros2 topic hz /lidar/scan       # how fast a topic publishes
ros2 topic info /cmd_vel        # type + how many pubs/subs
ros2 interface show nav_msgs/msg/Odometry   # the fields of a message type
```

`/wheel/odometry` appears after Lesson 5; localization, planning, and control
topics appear one at a time as you complete Lessons 9–11.

For a first pass, focus on nodes, topics, messages, building, and launching.
Services, actions, and components are recognition-level ideas until later lessons.

## 3. Services (request/response)

Topics are a continuous stream. Sometimes you want a one-off **"do this and tell
me when done"** — that's a **service** (request → response, like a function call
over the network). This repo leans on topics far more than services, but you
should know they exist:

```sh
ros2 service list
ros2 service call /some_service std_srvs/srv/Trigger
```

(ROS 2 also has **actions** for long-running goals with feedback, e.g. "drive to
this pose, tell me your progress." Nav2 uses these heavily; our simplified stack
uses plain topics to stay readable.)

## 4. Parameters

A **parameter** is a named, typed setting a node reads at startup (and sometimes
at runtime). Parameters are how you tune a node without recompiling.

In the Lesson 5 `wheel_odometry.cpp`, the constructor declares parameters with defaults:

```cpp
wheel_radius_      = declare_parameter("wheel_radius", 0.075);
wheel_separation_  = declare_parameter("wheel_separation", 0.34);
```

(`declare_parameter` both registers the parameter *and* returns its effective
value — the default unless someone overrides it — which is why it reads like a
plain assignment.)

Inspect a node that the Lesson 1 simulator actually starts:

```sh
ros2 param list /robot_state_publisher
ros2 param get /robot_state_publisher use_sim_time
```

Lesson 5 shows how to override wheel-odometry parameters when that node exists.

Bigger configs live in YAML files — e.g. `src/ekf_localization/config/ekf.yaml`
is *entirely* parameters (Lesson 7).

## 5. The TF system (preview)

Robots have many coordinate frames — the map, the wheels, the lidar. **TF2** is
the subsystem that tracks how every frame relates to every other, over time, so
any node can ask "where is the lidar relative to the map right now?" This is
important enough to get its own lesson (Lesson 2). For now just know: TF is a
special, time-stamped topic system for coordinate frames.

## 6. Packages, the workspace, and `colcon`

Each folder under `src/` is a **package**: a unit of code with a name, dependencies
(`package.xml`), and build rules (`CMakeLists.txt` for C++, or pure Python).

All the packages together, under one root with a `src/` folder, form a
**workspace**. You compile the whole workspace with **colcon**:

```sh
colcon build --symlink-install   # build everything in src/
source install/setup.bash        # make the built packages available to ros2
```

- `colcon build` produces `build/`, `install/`, and `log/` directories (git-ignored).
- `source install/setup.bash` adds your freshly built packages to the environment
  so `ros2 run` / `ros2 launch` can find them. **You must re-source after every
  build in a new terminal.**
- `--symlink-install` symlinks Python/launch/config files instead of copying, so
  edits to those take effect without rebuilding (C++ changes still need a rebuild).

If `ros2 launch my_package ...` says "package not found," 9 times out of 10 you
forgot to `source install/setup.bash`.

## 7. Launch files

Starting nodes one by one is tedious. A **launch file** starts many at once with
the right parameters and remappings. (A **remapping** renames a topic at launch
time without touching the code — e.g. a node that publishes a generic `/odom` can
be told to publish `/wheel/odometry` instead, so different nodes line up.) Launch
files are written in Python and live in each package's `launch/` folder.

After Lesson 5, open `src/wheel_odometry/launch/wheel_odometry.launch.py`. It
starts the wheel odometry node and passes `use_sim_time`. The big one,
`src/robonav_training_bringup/launch/navigation.launch.py`, starts the entire
stack (Lesson 12).

```sh
# Available immediately:
ros2 launch robonav_training_bringup sim.launch.py

# Available after Lesson 5 creates and builds wheel_odometry:
ros2 launch wheel_odometry wheel_odometry.launch.py
```

## 8. Components (composable nodes)

A normal node is its own process. A **component** is a node compiled as a shared
library that can be loaded into a shared **container** process alongside other
components. Multiple components in one container can pass messages without copying
them over the network — lower latency, less overhead.

This repo deliberately uses components. From the top-level `README.md`: five of the
nodes "run as composable components in one container." In the code you'll see this
line at the bottom of each C++ node, e.g. in `wheel_odometry.cpp`:

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(robonav_training::WheelOdometry)
```

> **Project convention:** all nodes are written in C++ as composable components;
> only the launch files are Python. Keep to that if you add code.

You don't need to master components now — just recognize the registration macro
and know "component = a node that can share a process."

## 9. `use_sim_time`: a concept you'll see everywhere

When running in simulation, ROS nodes should use the **simulator's** clock, not
your computer's wall clock, so everything stays in sync (and so you can pause
time). Setting the parameter `use_sim_time: true` tells a node to read time from
the `/clock` topic that **Gazebo** (the physics simulator this repo uses — set up
in Lesson 1, sensors in Lesson 6) publishes. Almost every launch file in this repo
sets it. Lesson 6 and 12 revisit why this matters.

## Hands-on

You cannot run ROS until Lesson 1 starts the container, but you can inspect the
provided map server now:

1. Open `src/map_server/src/map_server.cpp`. Find the parameter declaration,
   publisher creation, and `publish(...)` call.
2. Open `src/map_server/launch/map_server.launch.py`. Find the package, plugin,
   node name, and parameters passed to the component.
3. Open `src/map_server/CMakeLists.txt`. Find the library target, dependency list,
   component registration, and install rules.

You will create the same package/node/launch/build layers yourself in Project 0
and Lesson 5.

## Video supplement

- **10 things you need to know about ROS! | Getting Ready to Build Robots with ROS #4** (Articulated Robotics)

## Check yourself

- What is the difference between a topic and a service?
- Why does publish/subscribe decouple the planner from the controller?
- What does `source install/setup.bash` do, and when must you re-run it?
- What's a component, and why does this repo prefer them?

## Recap

ROS 2 = many small **nodes** talking over named **topics** (carrying typed
**messages**), tuned by **parameters**, started by **launch files**, built into a
**workspace** by **colcon**. Coordinate frames get their own system (**TF**, next
lesson). Everything else in this course is specific nodes plugged into this fabric.

➡️ **Next:** [Lesson 1 — Terminal & Docker](lesson-01-terminal-and-docker.md), so
you can actually run all of this.
