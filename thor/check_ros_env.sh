#!/usr/bin/env bash
set -uo pipefail

FAILED=0
WARNED=0

fail() {
  echo "❌ $*"
  FAILED=1
}

warn() {
  echo "⚠️ $*"
  WARNED=1
}

ok() {
  echo "✓ $*"
}

echo "=========================================="
echo "   MASt3R-SLAM ROS 2 Environment Check"
echo "=========================================="

echo "[1/9] OS & Kernel"
grep -E "PRETTY_NAME|VERSION_ID" /etc/os-release || true
echo "Kernel: $(uname -r)"

echo "[2/9] ROS Distribution"
ROS_DISTRO_EXPECTED="${ROS_DISTRO:-humble}"
echo "ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "Expected ROS distro: $ROS_DISTRO_EXPECTED"

echo "[3/9] ROS Environment"
ROS_SETUP="/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  set +u
  source "$ROS_SETUP"
  set -u
  ok "Sourced $ROS_SETUP"
else
  fail "$ROS_SETUP not found"
fi

echo "[4/9] ROS Commands"
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 command available: $(command -v ros2)"
  ros2 --help | head -n 3 || true
else
  fail "ros2 command not found"
fi

if command -v colcon >/dev/null 2>&1; then
  ok "colcon available: $(command -v colcon)"
else
  fail "colcon command not found"
fi

echo "[5/9] Key ROS 2 Packages"
if command -v ros2 >/dev/null 2>&1; then
  REQUIRED_PACKAGES=(rclpy sensor_msgs geometry_msgs std_msgs cv_bridge rosbridge_server foxglove_bridge)
  for package in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg prefix "$package" >/dev/null 2>&1; then
      echo "  ✓ $package: $(ros2 pkg prefix "$package")"
    else
      warn "$package not found"
    fi
  done
else
  fail "Cannot check ROS packages because ros2 is unavailable"
fi

echo "[6/9] Python ROS Imports"
python3 - <<'PY' || FAILED=1
modules = [
    ("rclpy", "rclpy"),
    ("sensor_msgs", "sensor_msgs"),
    ("geometry_msgs", "geometry_msgs"),
    ("std_msgs", "std_msgs"),
    ("cv_bridge", "cv_bridge"),
    ("roslibpy", "roslibpy"),
]
failed = False
for name, module in modules:
    try:
        mod = __import__(module)
        path = getattr(mod, "__file__", "built-in")
        print(f"  ✓ {name}: {path}")
    except Exception as exc:
        failed = True
        print(f"  ✗ {name}: {exc}")
if failed:
    raise SystemExit(1)
PY

echo "[7/9] Workspace Detection"
ROS_WS=""
for candidate in /workspace/thor/ros_ws /workspace/thor/ros2_ws /workspace/ros_ws /workspace/ros2_ws; do
  if [ -d "$candidate" ]; then
    ROS_WS="$candidate"
    break
  fi
done

if [ -n "$ROS_WS" ]; then
  ok "Workspace found: $ROS_WS"
  if [ -f "$ROS_WS/install/setup.bash" ]; then
    set +u
    source "$ROS_WS/install/setup.bash"
    set -u
    ok "Sourced $ROS_WS/install/setup.bash"
  else
    warn "$ROS_WS/install/setup.bash not found; workspace may not be built yet"
  fi
  if [ -d "$ROS_WS/src" ]; then
    echo "Workspace packages:"
    find "$ROS_WS/src" -mindepth 1 -maxdepth 2 -name package.xml -print | sed 's#/package.xml$##' | sed 's#^#  - #' | head -n 30
  else
    warn "$ROS_WS/src not found"
  fi
else
  fail "No ROS workspace found under /workspace/thor or /workspace"
fi

echo "[8/9] Network Configuration"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set (default 0)}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
if command -v ip >/dev/null 2>&1; then
  ip addr show | grep -E "inet " | head -n 8 || true
else
  warn "ip command not available"
fi

echo "[9/9] Common Runtime Mounts"
for path in /tmp/.X11-unix /dev/shm /workspace/thor; do
  if [ -e "$path" ]; then
    ok "$path exists"
  else
    warn "$path not found"
  fi
done
if [ -d /tmp/ipc_socket ]; then
  ok "/tmp/ipc_socket exists"
else
  warn "/tmp/ipc_socket not found; only relevant if your launch flow needs it"
fi

if [ "$FAILED" -ne 0 ]; then
  echo "=========================================="
  echo "❌ ROS 2 Environment Check Failed"
  echo "=========================================="
  exit 1
fi

if [ "$WARNED" -ne 0 ]; then
  echo "=========================================="
  echo "⚠️ ROS 2 Environment Check Passed With Warnings"
  echo "=========================================="
else
  echo "=========================================="
  echo "✅ ROS 2 Environment Check Passed"
  echo "=========================================="
fi
