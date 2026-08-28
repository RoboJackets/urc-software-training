# Lesson 1 — Terminal & Docker

> **Goal:** Get comfortable in a Linux terminal, understand what Docker is and why
> we use it, and start this repo's container so you can build and run the starting
> simulator from the TigerVNC desktop.

## Where this lives in the repo

- `start.sh`, `start.ps1`, `start.bat` — one-command launchers (mac/Linux, Windows
  PowerShell, Windows CMD).
- `compose.yml` — the top-level Docker Compose file.
- `docker/ros2-humble-vnc/Dockerfile` — the recipe that builds the container image.
- `docker/ros2-humble-vnc/docker-compose.yml`, `start-vnc.sh`, `README.md` — the
  VNC desktop setup inside the container.
- `README.md` (repo root) — the canonical run instructions.

## Part A — The terminal

Robotics development happens in a **terminal** (a.k.a. shell, command line). For
this course, run lesson commands from a terminal inside the TigerVNC desktop. That
terminal is already inside the container, where the shell is `bash`: you type a
command, it runs, it prints output.

### Survival commands

| Command | What it does |
| --- | --- |
| `pwd` | print working directory (where am I?) |
| `ls` / `ls -la` | list files / list all with details |
| `cd path` | change directory; `cd ..` goes up, `cd ~` goes home |
| `cat file` | print a file |
| `less file` | scroll a file (`q` to quit) |
| `nano file` | edit a file (this container has `nano`) |
| `mkdir name` | make a directory |
| `cp a b` / `mv a b` / `rm a` | copy / move / remove |
| `grep text file` | search inside files |
| `find . -name '*.cpp'` | find files by pattern |
| `clear` | clear the screen |
| Tab | autocomplete (use constantly) |
| ↑ / ↓ | scroll through previous commands |
| `Ctrl-C` | stop the running program |

### Paths, env vars, and sourcing

- **Absolute path** starts at root: `/workspace/src`. **Relative path** is from
  where you are: `src/wheel_odometry`.
- An **environment variable** is a named value the shell carries, like `$HOME` or
  `$ROS_DISTRO`. Print one with `echo $ROS_DISTRO` (→ `humble`).
- **Sourcing** a script (`source file.bash` or `. file.bash`) runs it *in your
  current shell* so it can set env vars that stick. That's why
  `source install/setup.bash` (Lesson 0) works the way it does.

### A few ROS-specific terminal commands (and what they mean)

You met these in Lesson 0; here's the "what's actually happening" version.

```sh
ros2 topic list           # ask the ROS graph for all active topic names
ros2 topic echo /imu/data # subscribe to a topic and print each message
ros2 topic hz /lidar/scan # measure publish rate (Hz = messages per second)
ros2 node list            # all running nodes
ros2 node info /robot_state_publisher  # a node started by the Lesson 1 simulator
ros2 run <pkg> <exe>      # run one executable from a package
ros2 launch <pkg> <file>  # run a launch file (starts many nodes)
ros2 param list           # all parameters of all nodes
rviz2                     # open RViz, the 3D visualizer
colcon build --symlink-install   # compile the workspace
```

`ros2 ...` works because the container's `.bashrc` sources
`/opt/ros/humble/setup.bash` automatically (see the `Dockerfile`). Your *own*
packages only appear after you also source `install/setup.bash`.

## Part B — Docker

### The problem Docker solves

ROS 2 Humble needs Ubuntu 22.04 plus dozens of exact library versions (Gazebo,
`robot_localization`, Eigen, `ros2_control`, …). Installing all of that directly on
your Mac or Windows machine is painful and breaks easily. **Docker** packages an
entire Linux environment — OS, libraries, tools — into an **image** that runs
identically on any machine. Everyone on the team gets the *same* environment.

### Core Docker vocabulary

- **Image** — a frozen, read-only snapshot of a filesystem + setup. Built from a
  **Dockerfile** (a recipe). Our image is `urc-ros2-humble-vnc:latest`.
- **Container** — a running instance of an image (like an object is an instance of
  a class). You can start, stop, and delete containers; the image stays.
- **Dockerfile** — the build recipe. Each line is a step (`FROM`, `RUN`, `COPY`…).
- **Volume / bind mount** — a folder shared between your real machine and the
  container, so files survive container restarts and you can edit them with your
  normal editor.
- **Docker Compose** — a tool that reads a `compose.yml` to start containers with
  all their settings (ports, mounts, env vars) in one command.

### Read our Dockerfile

Open `docker/ros2-humble-vnc/Dockerfile`. Walk the key steps:

```dockerfile
FROM ubuntu:22.04                 # start from a base Ubuntu image
ENV ROS_DISTRO=humble ...         # set environment variables
RUN apt-get install -y ... xfce4 tigervnc-standalone-server ...  # desktop + VNC
RUN curl ... packages.osrfoundation.org ...                      # Gazebo packages
RUN ... apt-get install -y ros-humble-desktop ros-humble-ros-gz \
        ros-humble-robot-localization ros-humble-ros2-control ...  # ROS + sim
RUN useradd ... ros               # create a non-root user
COPY start-vnc.sh /usr/local/bin/ # add the VNC startup script
CMD ["/usr/local/bin/start-vnc.sh"]   # what runs when the container starts
```

Things to notice:

- `FROM ... RUN ... COPY ... CMD` is the standard shape of a Dockerfile.
- It installs **exactly** the ROS packages this project needs — including
  `ros-humble-robot-localization` (the EKF, Lesson 7) and `ros-humble-ros-gz`
  (the Gazebo bridge, Lesson 6). Gazebo Fortress packages come from the official
  OSRF package repository configured immediately before the ROS repository. When
  you add a new dependency, you must add it here too — that's the rule in
  `AGENTS.md`.
- The last lines of the `.bashrc` setup auto-source ROS and (if present) your
  workspace's `install/setup.bash`.

### Read `compose.yml`

Open the root `compose.yml`:

```yaml
services:
  ros2-humble-vnc:
    build: { context: ./docker/ros2-humble-vnc, dockerfile: Dockerfile }
    platform: linux/amd64     # required Humble/Gazebo packages are amd64-only
    image: urc-ros2-humble-vnc:latest
    ports:
      - "5901:5901"          # expose the VNC port to your machine
    volumes:
      - .:/workspace         # mount this repo at /workspace inside the container
    shm_size: "1gb"          # shared memory (Gazebo/RViz need it)
```

The line `- .:/workspace` is the most important one: **your repo on disk appears
at `/workspace` inside the container.** Edit files normally on your host; build and
run them inside the container. Nothing is lost when the container stops.

### Part C — VNC: a graphical desktop in a box

Gazebo and RViz are graphical apps, but a container has no screen. The fix in this
repo: the container runs a lightweight **XFCE desktop** served over **VNC** (a
remote-desktop protocol). You connect a VNC *viewer* on your real machine to
`localhost:5901` and see a Linux desktop where Gazebo and RViz run.

These programs have different jobs. **Gazebo** is the physics simulator: it owns
the world, collisions, sensors, and the robot's actual simulated motion. **RViz**
is a ROS visualizer: it draws whatever ROS topics and TF frames are publishing,
but it does not simulate physics. Keep both open during the course. If the robot
moves in Gazebo but not RViz, inspect TF; if it appears in RViz but never moves in
Gazebo, inspect `/cmd_vel` and the controllers.

- Password: `ros` (set in `compose.yml` / `Dockerfile`).
- `start-vnc.sh` is the script that boots the desktop when the container starts.

## Hands-on — start the robot

Follow the repo `README.md`. Short version:

1. **Install Docker** (Docker Desktop on Mac/Windows; Docker Engine + Compose
   plugin on Linux) and a **TigerVNC viewer**.

2. **Start the container** from the repo root:

   ```sh
   ./start.sh          # macOS / Linux
   ```
   ```powershell
   .\start.ps1         # Windows PowerShell
   ```
   The first run builds the image (downloads a lot — be patient). Later runs are
   fast. Apple Silicon Macs use amd64 emulation because the required Humble
   Gazebo packages are not currently published for arm64, so their first build
   and simulator startup take longer.

3. **Connect** your TigerVNC viewer to `localhost:5901`, password `ros`. You should
   see a desktop.

4. **Open Terminal Emulator inside the TigerVNC desktop.** This is the normal place
   to run lesson commands, and it should start at `/workspace`.

   If you prefer using your own host terminal, open an equivalent container shell
   with:

   ```sh
   docker compose exec ros2-humble-vnc bash
   ```

5. **Build the workspace** in that VNC terminal or container shell:

   ```sh
   colcon build --symlink-install
   source install/setup.bash
   ```

   The Lesson 1 checkpoint is only the robot simulation: Gazebo, RViz, sensors, and
   teleop. The navigation stack is built up in later lessons.

6. **Launch the simulation:**

   ```sh
   ros2 launch robonav_training_bringup sim.launch.py
   ```

   Gazebo and RViz open on the VNC desktop. You should see the robot in both. RViz
   starts with Fixed Frame `base_link` because the global `map` frame is introduced
   later by the navigation stack; Lesson 12 switches RViz to `map`.

   Spend a minute distinguishing the two windows: orbit the camera around the
   physical world in Gazebo, then switch to RViz and expand **RobotModel** and
   **TF** in the Displays panel. Green `Status: Ok` entries confirm RViz is
   receiving the robot description and transforms. The **Fixed Frame** field at
   the top of Global Options controls which coordinate frame RViz holds still.

7. **Drive it.** Open a *second* Terminal Emulator window inside TigerVNC and get
   into the workspace first. If you are using host terminals instead, start another
   shell with `docker compose exec ros2-humble-vnc bash`.

   ```sh
   cd /workspace          # the bind-mounted repo; usually already the default dir
   source install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

   "Teleop" = teleoperation: this node turns your keypresses into `/cmd_vel`
   velocity commands. Use the keys it prints to drive, and watch the robot move in
   Gazebo. Every new shell needs its own `source install/setup.bash`.

8. **Stop everything** when done. Run this from a **host terminal at the repo
   root**, not from the VNC/container terminal (the container cannot shut itself
   down with Docker Compose):

   ```sh
   docker compose down
   ```

## Common gotchas

- **"package not found"** → you forgot `source install/setup.bash` in that shell.
- **Graphical app won't open / black screen** → reconnect the VNC viewer; make sure
  graphical apps were launched from a VNC terminal or container shell, not directly
  on your host.
- **Changed a `.launch.py`/YAML and nothing changed** → those are symlinked
  (`--symlink-install`), so just relaunch. **Changed C++** → re-run `colcon build`.

## Video supplement
- **Docker in 100 Seconds** (Fireship)

## Check yourself

- What's the difference between an image and a container?
- What does the `- .:/workspace` line in `compose.yml` do, and why does it matter?
- Why does this setup need VNC at all?

## Recap

Docker gives everyone the identical ROS 2 Humble environment as a **container**
built from a **Dockerfile**; **Compose** starts it with the repo bind-mounted at
`/workspace`; **VNC** gives you a desktop for Gazebo/RViz. Inside, you live in a
**bash** terminal, build with **colcon**, and drive ROS with `ros2 ...` commands.

➡️ **Next:** [Practice — Your First ROS 2 C++ Node](first-cpp-node.md), then
[Lesson 2 — Coordinate Transforms & TF2](lesson-02-coordinate-transforms-and-tf.md).
