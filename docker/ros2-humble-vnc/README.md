# ROS 2 Humble TigerVNC Container

ROS 2 Humble on Ubuntu 22.04 with an XFCE desktop over TigerVNC.

## Install

Install Docker:

- macOS/Windows: install Docker Desktop.
- Linux: install Docker Engine with the Docker Compose plugin.

Install TigerVNC Viewer:

- macOS: `brew install --cask tigervnc-viewer`
- Windows: install TigerVNC from <https://tigervnc.org/>
- Linux: install TigerVNC Viewer with your package manager.

## Start

From the repository root:

```sh
./start.sh
```

Windows PowerShell:

```powershell
.\start.ps1
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

Run project commands from a terminal inside the TigerVNC desktop. If you want to use
your own host terminal instead, open an equivalent shell with `docker compose exec
ros2-humble-vnc bash`.

## Useful Commands

Open **Terminal Emulator** inside the TigerVNC desktop. Host-terminal alternative:

```sh
docker compose exec ros2-humble-vnc bash
```

Build the mounted ROS workspace from that container shell:

```sh
colcon build --symlink-install
source install/setup.bash
```

Launch Gazebo and RViz:

```sh
ros2 launch robonav_training_bringup sim.launch.py
```

Drive with teleop:

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Run RViz:

```sh
rviz2
```

Stop and remove the container:

```sh
docker compose down
```
