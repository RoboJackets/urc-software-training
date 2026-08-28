#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

VNC_HOST_PORT="${VNC_HOST_PORT:-5901}"
VNC_PASSWORD="${VNC_PASSWORD:-ros}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not on PATH." >&2
  exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "Docker Compose is not available. Install Docker Desktop or the Docker Compose plugin." >&2
  exit 1
fi

echo "Building image and starting ROS 2 Humble VNC container..."
docker compose up --build -d

echo
echo "Container is running."
echo "VNC address: localhost:${VNC_HOST_PORT}"
echo "VNC password: ${VNC_PASSWORD}"
echo
echo "Run lesson commands in Terminal Emulator inside the TigerVNC desktop."
echo "Host-terminal alternative:"
echo "  docker compose exec ros2-humble-vnc bash"
echo
echo "Stop it with:"
echo "  docker compose down"
