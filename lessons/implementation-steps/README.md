# Implementation Steps

These files are the larger coding track for Lessons 4–12. The short
[Project 0](../first-cpp-node.md) comes earlier and is self-contained.
The main lesson pages explain the concepts; the implementation steps tell a
student what to build in a starter repo.

Use one loop throughout the course: read the numbered lesson, complete its
implementation page, pass its build/runtime check, then move to the next lesson.
Later projects assume every earlier checkpoint works.

This repository is the student starter. Packages built from scratch are absent;
later algorithm files contain TODO bodies that compile safely. Follow each lesson
in order and do not copy completed implementations from the separate answer-key
repository until after attempting the exercise.

## Build metadata (read once)

`package.xml` tells ROS which packages and runtime tools a project depends on.
`CMakeLists.txt` tells the compiler what to build and tells ROS which headers,
libraries, configs, and launch files to install. These files are mostly boilerplate
for this course, so each project reference now includes the exact versions to use.
Copy those two files into the project, then focus on the C++/YAML/launch work the
lesson assigns. Understand what each file controls; you do not need to invent the
build metadata.

## Build Discipline

- Add source files and headers as the lesson asks for them.
- Do not wire a new C++ target into `CMakeLists.txt` until that lesson is ready to
  compile that target.
- Once a feature is wired into CMake, run `colcon build --symlink-install` and fix
  compile errors before moving on.
- Source every new terminal after a build with `source install/setup.bash`.
- If `ros2` is not found, first run `source /opt/ros/humble/setup.bash`; then
  source `install/setup.bash` after the workspace has been built.
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

Each implementation page points to one numbered project sheet in
[`lessons/reference`](../reference/README.md). Keep that sheet open while working.
Its scaffolds are intentionally incomplete: use the requirements to fill the
gaps instead of searching for a complete file to copy.

## What Students Build

By the end of the implementation track, students will have written:

- a small publisher/subscriber node in Project 0
- a package from scratch
- a header file
- a composable C++ node
- a launch file
- subscribers
- publishers
- inspect and debug TF lookups and broadcasts
- core robotics algorithms: odometry, sensor fusion configuration, localization,
  path planning, and path following

## Lesson Map

| Step | Implementation role |
| --- | --- |
| Project 0 | Build a tiny publisher/subscriber node from scratch |
| Lesson 4 | Paper math only; prepares the wheel odometry implementation |
| Lesson 5 | Build `wheel_odometry` from scratch |
| Lesson 6 | Inspect sensors; no new package required |
| Lesson 7 | Build the `ekf_localization` config package and launch file from scratch |
| Lesson 8 | Use the provided map server; no coding changes |
| Lesson 9 | Implement the particle-filter motion update with scaled Gaussian noise |
| Lesson 10 | Implement the A* search algorithm |
| Lesson 11 | Implement the pure pursuit command computation |
| Lesson 12 | Wire, run, and debug the full stack |
