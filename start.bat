@echo off
setlocal

cd /d "%~dp0"

if "%VNC_HOST_PORT%"=="" set "VNC_HOST_PORT=5901"
if "%VNC_PASSWORD%"=="" set "VNC_PASSWORD=ros"

where docker >nul 2>nul
if errorlevel 1 (
  echo Docker is not installed or is not on PATH.
  exit /b 1
)

docker compose version >nul 2>nul
if errorlevel 1 (
  echo Docker Compose is not available. Install Docker Desktop.
  exit /b 1
)

echo Building image and starting ROS 2 Humble VNC container...
docker compose up --build -d
if errorlevel 1 exit /b %errorlevel%

echo.
echo Container is running.
echo VNC address: localhost:%VNC_HOST_PORT%
echo VNC password: %VNC_PASSWORD%
echo.
echo Run lesson commands in Terminal Emulator inside the TigerVNC desktop.
echo Host-terminal alternative:
echo   docker compose exec ros2-humble-vnc bash
echo.
echo Stop it with:
echo   docker compose down
