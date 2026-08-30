# Lesson 12 — Putting It All Together (Bringup)

> **Goal:** See how every package you've studied is wired into one running system by
> the `robonav_training_bringup` launch files, understand the end-to-end data flow,
> and learn how to run and debug the complete navigation stack.

## Where this lives in the repo

- `src/robonav_training_bringup/launch/sim.launch.py` — starts Gazebo, the robot,
  the bridge, the controllers, and RViz.
- `src/robonav_training_bringup/launch/navigation.launch.py` — starts the
  localization + planning + control stack.
- `src/robonav_training_bringup/config/gz_bridge.yaml` — the sensor bridge (Lesson 6).
- `src/robonav_training_bringup/config/robonav_training.rviz` — the RViz layout.
- `src/robonav_training_bringup/worlds/training_world.sdf`, `empty.sdf` — worlds.

"Bringup" is the ROS term for the package that *brings up* a whole system — it owns
no algorithms, just the launch files and configs that compose everyone else.

## The two-launch design

You run the system in two commands, usually in two Terminal Emulator windows inside
TigerVNC:

```sh
# Terminal 1 — the simulated robot and world:
ros2 launch robonav_training_bringup sim.launch.py

# Terminal 2 (once the sim is up) — the navigation brain:
ros2 launch robonav_training_bringup navigation.launch.py
```

Splitting them is deliberate: you can restart the navigation stack (where you're
likely iterating) without tearing down and reloading the whole simulator.

## What `sim.launch.py` starts

This is the "body" — the robot and its simulated world. Launch arguments include
`use_sim_time` (default true), `world` (default `training_world.sdf`), `robot_name`,
`rviz_config`, and `bridge_config`. It brings up:

1. **Gazebo** with the chosen world (publishes `/clock`).
2. **`robot_state_publisher`** — expands the URDF (Lesson 3) and publishes the
   static body TF (`base_footprint → base_link → sensors`).
3. **`ros_gz_bridge`** — bridges `/clock`, `/lidar/scan`, `/imu/data` (Lesson 6).
4. **`spawn`** — injects the robot into Gazebo (delayed ~2 s so Gazebo is ready).
5. **`joint_state_broadcaster`** and **`diff_drive_controller`** — the ros2_control
   controllers (Lesson 3), started after the robot spawns. These give you
   `/joint_states` and accept `/cmd_vel`.
6. **RViz** with the project layout.

After this you have a drivable robot: `ros2 run teleop_twist_keyboard
teleop_twist_keyboard` works, and the lidar/IMU publish. But the robot doesn't know
where it is or how to navigate yet — that's the second launch.

## What `navigation.launch.py` starts

This is the "brain" — everything from Lessons 5, 7, 8, 9, 10, 11. Launch args:
`use_sim_time` (true) and `map` (default `map_server/maps/training_map.yaml`). It
brings up:

1. **`ekf_localization`** — `robot_localization`'s EKF, run as its **own process**
   (Lesson 7). Subscribes `/wheel/odometry` + `/imu/data`; publishes
   `/odometry/filtered` and the `odom → base_footprint` TF.

2. A **component container** (`robonav_navigation_container`) hosting **five
   composable components** in one process (Lesson 0 — lower latency than five
   separate processes):

   | Component | Package | Lesson | Key I/O |
   | --- | --- | --- | --- |
   | `WheelOdometry` | wheel_odometry | 5 | `/joint_states` → `/wheel/odometry` |
   | `MapServer` | map_server | 8 | → `/map` (latched) |
   | `ParticleFilter` | particle_filter | 9 | `/odometry/filtered`, `/lidar/scan`, `/map` → `/amcl_pose`, TF `map→odom` |
   | `AStarPlanner` | a_star_planner | 10 | `/map`, `/goal_pose`, TF → `/plan` |
   | `PurePursuit` | pure_pursuit | 11 | `/plan`, TF → `/cmd_vel` |

This matches the table in the repo's top-level `README.md`. (The EKF is separate
because it's a third-party node, not one of our components.)

## The complete data flow

Read this loop top to bottom — it's the entire course in one diagram:

```
                    ┌─────────────── Gazebo (sim.launch.py) ───────────────┐
                    │  physics + world (training_world.sdf)                 │
                    └───┬───────────────┬───────────────┬──────────────────┘
            /joint_states           /lidar/scan      /imu/data   (+ /clock)
                    │                   │               │
            ┌───────▼────────┐          │       ┌───────▼────────┐
            │ wheel_odometry │          │       │                │
            │  (Lesson 5)    │          │       │                │
            └───────┬────────┘          │       │                │
              /wheel/odometry           │       │                │
                    │                   │       │                │
            ┌───────▼───────────────────┼───────▼────────┐
            │ ekf_localization (EKF, Lesson 7)            │  fuses wheels + IMU
            └───────┬─────────────────────────────────────┘
              /odometry/filtered  + TF odom→base_footprint
                    │                   │
            ┌───────▼───────────────────▼────────┐   ┌──────────────┐
            │ particle_filter (MCL, Lesson 9)     │◄──│ map_server   │
            │  scan + odom + map → pose           │   │ /map (L8)    │
            └───────┬─────────────────────────────┘   └──────┬───────┘
              /amcl_pose + TF map→odom                       /map
                    │                                          │
            (now the full TF map→odom→base_footprint exists)   │
                    │                                          │
   RViz 2D Goal ──► /goal_pose ──► ┌──────────────────────────▼───┐
                                   │ a_star_planner (Lesson 10)    │
                                   └───────────────┬───────────────┘
                                                 /plan
                                                   │
                                   ┌───────────────▼───────────────┐
                                   │ pure_pursuit (Lesson 11)       │
                                   └───────────────┬───────────────┘
                                                /cmd_vel
                                                   │
                                   ┌───────────────▼───────────────┐
                                   │ diff_drive_controller (L3)     │ → wheels move
                                   └────────────────────────────────┘
                                                   │
                                          back to /joint_states ↺
```

Every arrow is a topic or TF you can inspect from the terminal. Every box is a
lesson.

## Running the whole thing — the golden path

First complete the
[Lesson 12 implementation steps](implementation-steps/lesson-12-integration.md).
You are replacing the individual launches used in Lessons 5–11 with one reusable
navigation launch; the simulator remains separate so it can stay running while
you restart your code.

1. **Build & source** (Lesson 1), in a TigerVNC terminal or container shell:
   ```sh
   colcon build --symlink-install && source install/setup.bash
   ```
2. **Terminal 1:** `ros2 launch robonav_training_bringup sim.launch.py`
   — wait for Gazebo + RViz to fully come up and the robot to appear.
3. **Terminal 2:** `source install/setup.bash` then
   `ros2 launch robonav_training_bringup navigation.launch.py`. After it settles,
   use a third sourced terminal to run `ros2 node list` and `ros2 topic list`.
   Confirm the navigation nodes and topics below are present before continuing.
4. **Set up RViz for navigation — this is required, not optional.** The
   simulator-only layout deliberately starts with Fixed Frame `base_link`, which
   keeps Lesson 1 usable before a map exists. For the full stack, expand **Global
   Options** and set **Fixed Frame** to `map`. Click **Add** (bottom-left), use the
   **By topic** tab, and add Map (`/map`), LaserScan (`/lidar/scan`), PoseArray
   (`/particle_cloud`), and Path (`/plan`); add TF from **By display type**. The
   occupancy map, live lidar, particle cloud, planned path, and TF axes should now
   tell the same story in one view.
5. **Localize:** the particle cloud starts spread out and only **converges once the
   robot moves and the lidar sees distinctive geometry** — so drive with teleop
   (`ros2 run teleop_twist_keyboard teleop_twist_keyboard` in another sourced shell)
   to speed it up. If it will not converge, check that `/map`, `/lidar/scan`,
   `/odometry/filtered`, and TF are all active before debugging the scoring model.
6. **Navigate:** click **2D Goal Pose** (toolbar), then click-drag a goal in free
   space. The click sets position; RViz also sends the drag heading, but this
   training planner uses position only. It draws `/plan`; pure pursuit drives it;
   the robot eases to a stop at the goal.

## Debugging the stack (the skills that matter most)

When something doesn't work, inspect the graph — don't guess:

```sh
ros2 node list                       # are all nodes/components alive?
ros2 topic list                      # do all the topics exist?
ros2 topic hz /lidar/scan            # is the lidar flowing?
ros2 topic hz /imu/data              # is the IMU flowing?
ros2 topic echo /wheel/odometry      # is odometry moving when you drive?
ros2 topic echo /odometry/filtered   # is the EKF producing output?
ros2 topic echo /amcl_pose           # is localization producing a pose?
ros2 run tf2_tools view_frames       # writes frames.pdf in the current dir — is the tree complete (map→…→lidar)?
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan                # did the planner produce a path?
ros2 topic echo /cmd_vel             # is the controller commanding motion?
```

### Common failure → likely cause

| Symptom | Likely cause | Lesson |
| --- | --- | --- |
| `package not found` | forgot `source install/setup.bash` | 1 |
| C++ change had no effect | didn't `colcon build` (only YAML/launch are symlinked) | 1 |
| No `/lidar/scan` or `/imu/data` | bridge not up / sim not ready | 6 |
| TF says `map` → nothing | particle filter not publishing `map→odom` | 9 |
| Robot drifts and never corrects | particle cloud not converging (no scan/map match) | 9 |
| Goal set but no `/plan` | goal in obstacle/unknown, or no `map→base_footprint` TF | 10 |
| A* says `Start cell is blocked` | robot is within the inflated obstacle buffer; teleop into open space and retry | 10 |
| `/plan` exists but robot won't move | `/cmd_vel` not reaching `diff_drive_controller` | 3, 11 |
| Everything laggy/jumpy in time | `use_sim_time` mismatch across nodes | 0, 6 |

**Order matters:** start the sim first, let it settle, then navigation. Many "bugs"
are just race conditions from launching the brain before the body's TF and sensors
exist.

## Why the architecture is shaped this way

Step back and appreciate the design choices you've now seen end to end:

- **Many small nodes over named topics** → each piece is independently testable and
  replaceable (swap the simulator for real hardware and nothing else changes).
- **Pure algorithm cores + thin ROS wrappers** (A*, pure pursuit, particle filter,
  map loader) → the hard logic is unit-testable without ROS.
- **The `map / odom / base_footprint` TF split** → smooth local control *and*
  drift-free global accuracy without one transform doing both.
- **Components in one container** for the hot path, a separate process for the
  third-party EKF → performance where it matters, reuse where it helps.
- **Config-driven tuning** (params, YAML) → behavior changes without recompiling.

That's the same philosophy stated in the root `README.md`: simplicity,
readability, and separation of responsibility above all.

## Final Proficiency Check

The course is complete when you can do these without copying a completed
implementation:

1. Run the golden path above and successfully send the robot to three different
   goals.
2. Use `view_frames` to print the TF tree and label which node publishes each edge
   (cross-check against Lesson 2).
3. Deliberately break one thing (e.g. don't launch navigation, or set a goal in a
   wall) and use the debugging table to diagnose it from the terminal alone.
4. Pick one parameter from any lesson (`num_particles`, `inflation_radius`,
   `lookahead_distance`) and observe its end-to-end effect on the full run.
5. Exchange one implementation with a teammate. Review topic/frame contracts,
   input guards, parameter names, callback responsibilities, logs, and whether the
   acceptance check proves the claimed behavior. The author should make one small
   follow-up commit from the review.

## Video supplement

- [Making full robot navigation easy with Nav2 and ROS / Nav2 series** (Articulate Robots)](https://www.youtube.com/watch?v=jkoGkAd0GYk):
  shows an equivalent full stack coming together and being debugged in RViz.
- Official **Nav2** docs/videos (<https://docs.nav2.org/>) — the production-grade
  version of the simplified stack you just built; now you'll recognize every piece.

## Check yourself

- Why are `sim.launch.py` and `navigation.launch.py` separate?
- Which node publishes each TF edge in `map → odom → base_footprint → base_link`?
- Trace a single goal click all the way to wheel motion, naming every topic.
- Why does the EKF run as its own process while the other five are components?
- Given "goal set but robot won't move," what would you check, in order?

## Recap

`robonav_training_bringup` composes the entire course: `sim.launch.py` brings up the
robot, world, sensors, bridge, and controllers; `navigation.launch.py` brings up the
EKF plus a five-component container (odometry, map, particle filter, planner,
controller). Together they form the continuous **sense → localize → plan → control**
loop, all observable and debuggable from the terminal. You now understand — and can
run — a complete autonomous navigation stack, from `/joint_states` to `/cmd_vel` and
back.

🎉 **You've finished the course.** If you completed every implementation and
checkpoint—not only the reading—you now have working ROS 2 proficiency for a
small navigation stack: you can build packages, connect and launch nodes, reason
about TF and QoS, inspect the graph, and debug data flow end to end. Services,
actions, custom interfaces, hardware deployment, and Nav2 are the next layer.

Keep the skill by changing this stack, building a small personal project, or
using the same workflow in research. ROS becomes comfortable through repetition.
