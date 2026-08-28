# RoboNav ROS 2 Reference

This folder is a syntax and workflow reference for the starter version of the
RoboNav training. Use it when you know the concept you want, but forget the exact
ROS 2 C++ syntax, launch syntax, CMake wiring, message type, or terminal command.

It is intentionally practical. The examples match the style used in the starter
files that appear later in the course:

- C++17
- `rclcpp`
- composable nodes with `rclcpp_components`
- `.hpp` declarations plus `.cpp` implementations
- ROS-free algorithm cores where possible
- launch files that run components in containers

## Pages

| Page | Use it for |
| --- | --- |
| [C++ node patterns](cpp-node-patterns.md) | Minimal `.hpp`/`.cpp` scaffolds, then separate entries for smart pointers, publishers, subscribers, parameters, timers, TF, and ROS-free cores |
| [C++ algorithm patterns](cpp-algorithm-patterns.md) | vectors, safe indexing, math helpers, 2D rotations, priority queues, path reconstruction |
| [Messages and QoS](messages-and-qos.md) | message headers, field names, common message types, QoS choices |
| [Launch and parameters](launch-and-parameters.md) | standalone nodes, composable nodes, launch arguments, parameter passing |
| [CMake and package.xml](cmake-package.md) | adding dependencies, libraries, components, executables, installs |
| [Terminal commands](terminal-commands.md) | build, source, inspect topics/nodes/params/TF, debug common failures |

If the syntax on this page is new, complete
[Your First ROS 2 C++ Node](../first-cpp-node.md) before Lesson 2.

## How To Use This In Exercises

When a lesson asks you to add a node or feature, start with the smallest relevant
skeleton and add only the separate patterns that the application needs:

1. Add includes and class members in the `.hpp`.
2. Declare parameters in the constructor.
3. Create publishers/subscribers/timers in the constructor.
4. Implement callbacks in the `.cpp`.
5. Wire dependencies in `CMakeLists.txt` and `package.xml` once the feature is
   ready to compile.
6. Add or update the launch file.
7. Build, source, run, and inspect with terminal commands.

The important habit is not memorizing every line. The habit is knowing which file
owns which part of the ROS plumbing.

Do not copy an entire reference example unchanged. Use only the pieces that make
sense for the application you are building, then replace the example names,
types, topics, parameters, and behavior with the requirements from the lesson.
