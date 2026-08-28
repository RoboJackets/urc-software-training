# Implementation Steps

These files are the larger coding track for Lessons 4–12. The short
[first-node practice](../first-cpp-node.md) comes earlier and is self-contained.
The main lesson pages explain the concepts; the implementation steps tell a
student what to build in a starter repo.

This repository is the student starter. Packages built from scratch are absent;
later algorithm files contain TODO bodies that compile safely. Follow each lesson
in order and do not copy completed implementations from the separate answer-key
repository until after attempting the exercise.

## Build Discipline

- Add source files and headers as the lesson asks for them.
- Do not wire a new C++ target into `CMakeLists.txt` until that lesson is ready to
  compile that target.
- Once a feature is wired into CMake, run `colcon build --symlink-install` and fix
  compile errors before moving on.
- Source every new terminal after a build with `source install/setup.bash`.
- Keep graphical and ROS commands in TigerVNC terminals by default. A host terminal
  using `docker compose exec ros2-humble-vnc bash` is fine too.

## `package.xml` versus `CMakeLists.txt`

First-time ROS learners often mix up these two files:

- `package.xml` declares what the package depends on so ROS tooling, `rosdep`, and
  other packages understand it.
- `CMakeLists.txt` tells the compiler what to build, which dependencies a C++
  target uses, and which targets/config/launch files must be installed.

For a package created from scratch, `ros2 pkg create --dependencies ...` creates
both files and inserts the listed dependencies. You still need to add the library
or executable and install rules to CMake. Launch-only dependencies such as
`launch` and `launch_ros` should be added to `package.xml` as `<exec_depend>`
entries. Each from-scratch lesson below gives the exact entries and CMake shape.

After each acceptance check, review the change before moving on:

```sh
git diff --check
git diff
git status --short
```

Confirm that only intended files changed, topic/frame names and units are clear,
inputs are checked before indexing or division, and failures produce useful logs.
Run `ament_uncrustify path/to/changed.cpp` on changed C++ and use `--reformat` if
needed. Compiler warnings and formatting checks are part of completion, not
optional cleanup.
Add new files explicitly with `git add path`; `git commit -a` does not include
untracked files. Keep commits small enough that a teammate can review one idea at
a time.

## What Students Build

By the end of the implementation track, students will have written:

- a small composable publisher/subscriber node in the pre-Lesson-2 practice
- a package from scratch
- a header file
- a composable C++ node
- a launch file
- subscribers
- publishers
- TF lookups and TF broadcasts
- core robotics algorithms: odometry, sensor fusion configuration, localization,
  path planning, and path following

## Lesson Map

| Lesson | Implementation role |
| --- | --- |
| Practice | Build a tiny composable publisher/subscriber node from scratch |
| 4 | Paper math only; prepares the wheel odometry implementation |
| 5 | Build `wheel_odometry` from scratch |
| 6 | Inspect sensors; no new package required |
| 7 | Build the `ekf_localization` config package and launch file from scratch |
| 8 | Use the provided map server; no coding changes |
| 9 | Implement the particle-filter motion update with scaled Gaussian noise |
| 10 | Implement the A* search algorithm |
| 11 | Implement the pure pursuit command computation |
| 12 | Wire, run, and debug the full stack |
