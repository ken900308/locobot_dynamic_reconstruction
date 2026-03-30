#!/usr/bin/env bash
set -euo pipefail

# ROS2 Humble container runner for NVIDIA AGX Thor
# ROS-only image (Ubuntu 22.04 + ROS2 Humble)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Allow X11 for GUI
if command -v xhost > /dev/null 2>&1; then
  xhost +local:root > /dev/null 2>&1 || true
fi

# Build and start the container
echo ">>> Building and starting ROS2 Humble container..."
docker compose -f "docker/docker-compose.arm_humble_locobot.yml" build
docker compose -f "docker/docker-compose.arm_humble_locobot.yml" up -d mast3r_locobot_ros_humble

echo ">>> ROS2 Humble container is up"
