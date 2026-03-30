#!/usr/bin/env bash
set -euo pipefail

echo "=========================================="
echo "   Thor ROS2 Environment Check"
echo "=========================================="

echo "[1/8] OS & Kernel"
cat /etc/os-release | grep -E "PRETTY_NAME|VERSION_ID"
echo "Kernel: $(uname -r)"

echo "[2/8] ROS Distribution"
if [ -n "${ROS_DISTRO:-}" ]; then
  echo "ROS_DISTRO: $ROS_DISTRO"
else
  echo "ROS_DISTRO: not set"
fi

echo "[3/8] ROS Environment"
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  echo "✓ Sourced /opt/ros/humble/setup.bash"
else
  echo "⚠️ /opt/ros/humble/setup.bash not found"
fi

echo "[4/8] ROS Commands"
if command -v ros2 >/dev/null 2>&1; then
  echo "✓ ros2 command available"
  ros2 --help | head -n 3
else
  echo "❌ ros2 not found"
fi

if command -v colcon >/dev/null 2>&1; then
  echo "✓ colcon available"
else
  echo "❌ colcon not found"
fi

echo "[5/8] Key ROS2 Packages"
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg list 2>/dev/null | grep -E "^(rclcpp|rclpy|sensor_msgs|geometry_msgs|std_msgs|cv_bridge)$" || echo "  (some packages may be missing)"
else
  echo "Cannot check - ros2 command not available"
fi

echo "[6/8] Python CV Bridge"
python3 - <<'PY'
try:
    from cv_bridge import CvBridge
    print("  ✓ cv_bridge importable")
except ImportError:
    print("  ✗ cv_bridge not found")
PY

echo "[7/8] Network Configuration"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set (default 0)}"
ip addr show | grep -E "inet " | head -n 5

echo "[8/8] IPC Socket Mount"
if [ -d /tmp/ipc_socket ]; then
  echo "✓ /tmp/ipc_socket exists"
  ls -la /tmp/ipc_socket | head -n 5
else
  echo "⚠️ /tmp/ipc_socket not found"
fi

echo "=========================================="
echo "✅ ROS2 Environment Check Complete"
echo "=========================================="
