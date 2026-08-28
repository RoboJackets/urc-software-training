$ErrorActionPreference = "Stop"

Set-Location -Path $PSScriptRoot

if (-not $env:VNC_HOST_PORT) {
    $env:VNC_HOST_PORT = "5901"
}

if (-not $env:VNC_PASSWORD) {
    $env:VNC_PASSWORD = "ros"
}

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error "Docker is not installed or is not on PATH."
}

docker compose version *> $null

Write-Host "Building image and starting ROS 2 Humble VNC container..."
docker compose up --build -d

Write-Host ""
Write-Host "Container is running."
Write-Host "VNC address: localhost:$env:VNC_HOST_PORT"
Write-Host "VNC password: $env:VNC_PASSWORD"
Write-Host ""
Write-Host "Run lesson commands in Terminal Emulator inside the TigerVNC desktop."
Write-Host "Host-terminal alternative:"
Write-Host "  docker compose exec ros2-humble-vnc bash"
Write-Host ""
Write-Host "Stop it with:"
Write-Host "  docker compose down"
