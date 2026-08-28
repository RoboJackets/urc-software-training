# RoboNav Software Training

A hands-on course that teaches autonomous mobile-robot navigation by walking
through a **real, working ROS 2 Humble codebase**. Every concept is tied to code
you can read, build, and run in this repository.

**Estimated length: 7–8 weeks** when reading lessons and completing the coding
projects together. Project 0 is the short C++ practice after Lesson 1.

By the end you will understand — and have run — a full navigation stack: a robot
that knows where it is (**localization**), decides how to get somewhere
(**planning**), and drives there (**control**), all in simulation. Those three
words are the spine of the whole course:

- **Localization** — estimating where the robot is.
- **Planning** — choosing a route to a goal.
- **Control** — generating the motor commands to follow that route.

By the end of the coding track, you should have working proficiency with ROS 2 for
a small navigation system: creating packages and composable C++ nodes, connecting
publishers and subscribers, declaring parameters, wiring CMake and launch files,
using TF and QoS, separating algorithms from ROS plumbing, and debugging a
multi-node graph. Services, actions, custom messages, and production Nav2 are
follow-on topics, not hidden prerequisites.

## How to use this course

No ROS experience is required, but basic C++ is assumed. You should recognize
headers and source files, classes, functions, loops, references, `const`,
templates, and smart pointers such as `std::unique_ptr` and `std::shared_ptr`.
If those are unfamiliar, search YouTube for **"C++ basics for beginners"** and
**"modern C++ smart pointers"** and complete a beginner overview first. Project 0
then introduces the ROS-specific C++ patterns used throughout this repo.

1. Do the lessons **in order** — each builds on the last.
2. Keep the repo open beside you. Every lesson has a **"Where this lives in the
   repo"** section pointing at the exact files it explains.
3. Actually run the commands. Reading about a particle filter is nothing like
   watching the particle cloud snap onto the robot in RViz.
4. Where a lesson lists a **Video supplement**, those are optional but strongly
   recommended for the visual, intuition-heavy topics.

If this is your first time in the repository, use this sequence:

1. Read Lesson 0 for the ROS vocabulary.
2. Complete Lesson 1 to start Docker, connect to the desktop, build the workspace,
   and drive the simulated robot.
3. Complete the short [first C++ node practice](first-cpp-node.md). It removes the
   ROS/C++ syntax shock before the robotics-heavy implementations.
4. Continue through Lessons 2–11. For a coding lesson, read the concept page,
   complete its implementation page, and pass its checkpoint before moving on.
5. Use Lesson 12 to replace the individual project launches with one integrated
   navigation launch, then debug the complete system.

Reading alone is not course completion. Build every assigned project, run every
checkpoint, and diagnose at least one deliberately introduced failure. That is
what turns familiarity into working proficiency.

When you are building or editing code, keep the
[RoboNav ROS 2 Reference](reference/README.md) open and choose the sheet with the
same project number. Each project sheet collects the partial scaffolds, selected
code lines, and checks needed for that assignment. Use the shared terminal page
only when you need a build or inspection command.

Use the reference as a toolbox, not as an answer to copy from top to bottom. Take
only the pattern that fits the feature you are implementing, then deliberately
change the class, types, topics, parameters, and member names for your node.

The first-node practice is the small introductory coding assignment. Lessons 5,
7, and 9–12 have the larger coding track in
[Implementation Steps](implementation-steps/README.md). The main lesson explains
the robotics idea; the implementation step tells you what to create or fill in.
Lessons 4, 6, and 8 are concept/provided-code checkpoints that prepare later
implementation work.

This repo is the **student starter**. Some early packages are intentionally absent
because their lessons teach you to create them from scratch. Later packages keep
their ROS wrappers and build wiring but use compile-safe TODO implementations for
the lesson-owned algorithms. Create or fill in only the files and functions named
by each lesson's implementation step.

The workspace is meant to build even before unfinished features are implemented.
Do not add a new package to CMake, package metadata, or the full-stack launch until
the lesson tells you to wire it in and its current files compile.

## The robot you are building toward

This repo simulates a differential-drive robot (two driven wheels + a caster)
with a lidar on a tower and an IMU in its body. The full stack, from the
top-level `README.md`:

> **Don't worry if the names in the next two tables (`/wheel/odometry`, `map→odom`,
> "MCL", "occupancy grid", …) mean nothing yet.** Every one is explained in the
> lesson listed in its row. This page is a map, not the territory — skim it, then
> start at Lesson 0.

| Package | Produces | Introduced in |
| --- | --- | --- |
| `ros_cpp_practice` | `/practice/output` from a tiny node | Project 0 |
| `robonav_training_description` | The robot itself (URDF), sensors, controllers | 3, 6 |
| `robonav_training_common` | Shared math helpers (angles, grid math) | 2, 8 |
| `robonav_training_bringup` | Launches sim + the whole stack | 1, 12 |
| `wheel_odometry` | `/wheel/odometry` from wheel encoders | 4, 5 |
| `ekf_localization` | `/odometry/filtered` + TF `odom`→`base_footprint` | 7 |
| `map_server` | `/map` (the static occupancy grid) | 8 |
| `particle_filter` | `/amcl_pose` + TF `map`→`odom` (lidar MCL) | 9 |
| `a_star_planner` | `/plan` for a goal sent to `/goal_pose` | 10 |
| `pure_pursuit` | `/cmd_vel` to follow `/plan` | 11 |

## Lessons

| Order | Lesson | What you learn | Maps to |
| --- | --- | --- | --- |
| Lesson 0 | [ROS 2 Foundations](lesson-00-ros-foundations.md) | Nodes, topics, messages, services, params, components, colcon | whole repo |
| Lesson 1 | [Terminal & Docker](lesson-01-terminal-and-docker.md) | Shell basics, what Docker is, how this container works, VNC | `docker/`, `start.*` |
| Project 0 | [First ROS 2 C++ Node](first-cpp-node.md) | ROS C++ survival syntax; a small publisher/subscriber node | `ros_cpp_practice` |
| Lesson 2 | [Coordinate Transforms & TF2](lesson-02-coordinate-transforms-and-tf.md) | Frames, rotations, quaternions, the TF tree, angle wrapping | `robonav_training_common` |
| Lesson 3 | [URDF & the Robot Description](lesson-03-urdf-robot-description.md) | xacro, links, joints, sensors, ros2_control | `robonav_training_description` |
| Lesson 4 | [Differential-Drive Kinematics](lesson-04-diff-drive-kinematics.md) | How two wheels turn into motion; the forward-kinematics math | (theory for 5) |
| Lesson 5 | [Wheel Odometry](lesson-05-wheel-odometry.md) | Dead reckoning, integrating pose, why it drifts; implement the package from scratch | `wheel_odometry` |
| Lesson 6 | [Sensors: IMU & Lidar](lesson-06-sensors-imu-lidar.md) | What an IMU and a lidar measure, noise, the Gazebo bridge | `robonav_training_description`, bringup |
| Lesson 7 | [EKF & Sensor Fusion](lesson-07-ekf-sensor-fusion.md) | Bayesian filtering, the EKF, fusing wheels + IMU; implement the config package from scratch | `ekf_localization` |
| Lesson 8 | [Occupancy Grids & the Map Server](lesson-08-occupancy-grids-map-server.md) | Maps as grids, PGM/YAML format, world↔grid math | `map_server`, `robonav_training_common` |
| Lesson 9 | [Particle Filter (MCL)](lesson-09-particle-filter-mcl.md) | Monte Carlo Localization, likelihood fields, resampling; implement the noisy motion update | `particle_filter` |
| Lesson 10 | [A* Path Planning](lesson-10-a-star-planning.md) | Graph search, heuristics, obstacle inflation; implement A* search | `a_star_planner` |
| Lesson 11 | [Pure Pursuit Path Following](lesson-11-pure-pursuit.md) | Lookahead control, curvature, turning a path into `/cmd_vel`; implement the controller core | `pure_pursuit` |
| Lesson 12 | [Putting It All Together](lesson-12-bringup-full-stack.md) | The full launch graph, data flow, debugging the stack | `robonav_training_bringup` |

## How to do the hands-on sections (read this once)

A few things that apply to **every** lesson's "Hands-on" block, so they aren't
repeated each time:

- **Always work inside the container** (Lesson 1). For normal lessons, open terminal
  windows inside the TigerVNC desktop; if you prefer your own host terminal, use
  `docker compose exec ros2-humble-vnc bash` to get an equivalent container shell.
  **`source install/setup.bash` in every new terminal**, after building once with
  `colcon build --symlink-install`. If `ros2` can't find a package, you forgot to
  source.
  The container normally sources ROS 2 Humble for you; if `ros2` itself is not
  found, run `source /opt/ros/humble/setup.bash` first, then source the workspace.
- **You'll need several terminals at once** — typically one for the simulator, one
  for the navigation stack, and one or more for `ros2 topic echo`/teleop.
- **The runtime grows one project at a time.** Early lessons need only the
  simulator. Lessons 5–11 add their package's standalone launch to the processes
  from the previous checkpoint. Lesson 12 replaces those individual launches with
  `navigation.launch.py`. You should never need to skip a checkpoint and return
  later.
- **RViz** is the 3D viewer (Lesson 1). You interact with the robot mostly through
  the **2D Goal Pose** toolbar button (send a navigation goal, Lesson 10). The
  particle filter in this repo starts with global localization, so you localize by
  driving until the particle cloud converges.
- **Do not wire unfinished code into CMake or launch files.** For implementation
  lessons, create the files first, then add them to `CMakeLists.txt` and launch
  files only when that feature is ready to compile and run. Some starter packages
  in later lessons may already have compiling CMake/launch scaffolding; in those
  cases, fill only the lesson-designated missing file or function.

## The big picture (read this before Lesson 0)

Autonomous navigation is four questions, answered continuously:

1. **What does my robot look like, and where are its sensors?** → URDF (Lesson 3)
2. **Where am I?** → *Localization*: wheel odometry (5) + IMU, fused by an EKF (7),
   then corrected against a map by a particle filter (9).
3. **How do I get to the goal?** → *Planning*: A* search over the map (10).
4. **How do I actually drive that path?** → *Control*: pure pursuit (11).

Everything else (ROS, Docker, TF, grids) is the plumbing that lets those four
pieces talk to each other. Start with Lesson 0.
