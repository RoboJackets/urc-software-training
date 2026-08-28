# RoboNav Software Training

A hands-on introduction to autonomous mobile-robot navigation with ROS 2 Humble.
This repository is the **student starter** for a 13-lesson course plus one short
C++ node practice, covering ROS basics, coordinate frames, robot description,
localization, planning, control, and full-stack integration.

This course is meant to get you familiar with ROS 2 and comfortable working with
it. Like any skill, becoming genuinely good at ROS takes repetition. Personal
projects, research, and simply experimenting with this training repo—changing a
parameter, tracing a topic, or reading more of the code—will help tremendously.

After going through this file, start with the
[course overview and lesson list](lessons/README.md). Work through the lessons in
order; each lesson points to the code it explains and includes a hands-on
checkpoint. Lessons 5, 7, and 9–12 link to implementation steps for the packages,
functions, and launch wiring you will complete; the short first-node practice
comes immediately after Lesson 1.

The development environment is Ubuntu 22.04 with an XFCE desktop, Gazebo, RViz,
and ROS 2 Humble, all provided through Docker and TigerVNC.

## Course Scope

The course builds a simulated differential-drive navigation stack with:

- wheel odometry and IMU fusion using `robot_localization`'s EKF
- lidar localization against a static map using a particle filter
- A* path planning
- pure pursuit path following
- full-stack integration and debugging in Gazebo and RViz

The first-node practice is linked from the lesson list. The larger implementation
assignments are in
[lessons/implementation-steps](lessons/implementation-steps/README.md).

## Starter-repository build rule

The repository should build before every implementation lesson. Packages created
from scratch are intentionally absent until their lesson. Later packages contain
compile-safe TODO implementations, and the full navigation launch file stays
minimal until Lesson 12 wires completed components into it.

## Install

Install Docker:

- macOS/Windows: install Docker Desktop.
- Linux: install Docker Engine with the Docker Compose plugin.

Install TigerVNC Viewer:

- macOS: `brew install --cask tigervnc-viewer`
- Windows: install TigerVNC from <https://tigervnc.org/>
- Linux: install TigerVNC Viewer with your package manager.

## Start

macOS/Linux:

```sh
./start.sh
```

The first build downloads the complete ROS/Gazebo desktop and can take several
minutes. On an Apple Silicon Mac, Docker uses amd64 emulation because the required
ROS 2 Humble Gazebo packages are not available in the current arm64 repository;
the first build and Gazebo startup will therefore be slower than on an x86-64
machine.

Windows PowerShell:

```powershell
.\start.ps1
```

Windows Command Prompt:

```bat
start.bat
```

Connect TigerVNC Viewer to:

```text
localhost:5901
```

Password:

```text
ros
```

The repo is mounted at `/workspace` in the container.

Run the lesson commands from a terminal inside the TigerVNC desktop. That terminal
is already inside the container and starts in the mounted workspace. If you prefer
using your own host terminal, open the same kind of shell with `docker compose exec
ros2-humble-vnc bash`.

## Useful Commands

Open **Terminal Emulator** inside the TigerVNC desktop. Host-terminal alternative:

```sh
docker compose exec ros2-humble-vnc bash
```

Build the mounted ROS workspace from that container shell:

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Now you are ready to follow the lessons!

Stop and remove the container:

```sh
docker compose down
```
