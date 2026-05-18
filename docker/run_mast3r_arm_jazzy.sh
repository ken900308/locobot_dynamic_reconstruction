#!/usr/bin/env bash
set -euo pipefail

# ARM version of run_mast3r.sh for NVIDIA AGX Thor
# Uses official NVIDIA PyTorch base image: nvcr.io/nvidia/pytorch:25.08-py3

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Default parameters
DATASET_DEFAULT="datasets/tum/rgbd_dataset_freiburg1_desk"
CONFIG_DEFAULT="config/base.yaml"

DATASET="${1:-$DATASET_DEFAULT}"
CONFIG="${2:-$CONFIG_DEFAULT}"
EXTRA_ARGS="${3:-}"
IMAGE_NAME="mast3r-slam:arm64-thor-locobot"
CONTAINER_NAME="mast3r_locobot_jazzy"
COMPOSE_FILE="docker/docker-compose.arm_jazzy.yml"

# Set this to 1 after changing the Dockerfile and wanting a rebuild.
# Keep it at 0 to reuse the existing image for faster startup.
REBUILD_IMAGE=0

# Allow X11 for GUI
if command -v xhost > /dev/null 2>&1; then
  xhost +local:root > /dev/null 2>&1 || true
fi

# Build only when the image is missing, REBUILD_IMAGE=1, or FORCE_BUILD=1 is set.
if [ "${FORCE_BUILD:-$REBUILD_IMAGE}" = "1" ] || ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
  echo ">>> Building ARM image..."
  docker compose -f "$COMPOSE_FILE" build
else
  echo ">>> Reusing existing ARM image: $IMAGE_NAME"
fi

echo ">>> Starting ARM container..."
docker compose -f "$COMPOSE_FILE" up -d --no-build mast3r_locobot_jazzy

echo ">>> Setting up container environment..."
docker exec -it "$CONTAINER_NAME" env FORCE_SETUP="${FORCE_SETUP:-0}" bash -lc "
  set -euo pipefail

  SETUP_MARKER=/usr/local/share/mast3r_locobot_arm_jazzy_setup_complete
  if [ \"\$FORCE_SETUP\" != \"1\" ] && [ -f \"\$SETUP_MARKER\" ]; then
    echo '>>> Container environment already set up; set FORCE_SETUP=1 to reinstall'
    exit 0
  fi

  echo '>>> Fixing GUI Resources for MASt3R-SLAM'
  
  # Find Python site-packages directory
  SITE_PACKAGES=\$(python3 -c 'import site; print(site.getsitepackages()[0])')
  mkdir -p \${SITE_PACKAGES}/resources
  
  echo 'Copying in3d resources...'
  if [ -d '/workspace/thor/MASt3R-SLAM/thirdparty/in3d/resources' ]; then
    cp -r /workspace/thor/MASt3R-SLAM/thirdparty/in3d/resources/* \${SITE_PACKAGES}/resources/ 2>/dev/null || true
    echo '  ✓ in3d resources copied'
  fi
  
  echo 'Copying MASt3R-SLAM resources...'
  if [ -d '/workspace/thor/MASt3R-SLAM/resources' ]; then
    cp -r /workspace/thor/MASt3R-SLAM/resources/* \${SITE_PACKAGES}/resources/ 2>/dev/null || true
    echo '  ✓ MASt3R-SLAM resources copied'
  fi
  
  echo '✓ GUI Resources setup complete'

  echo '>>> Python/Environment check'
  python -V
  nvcc --version || true
  python -c 'import torch; print(\"torch:\", torch.__version__); print(\"CUDA available:\", torch.cuda.is_available()); print(\"GPU:\", torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\")'

  echo '>>> Entering project root directory'
  cd /workspace/thor/MASt3R-SLAM

  echo '>>> Upgrading packaging tools'
  python -m pip install -U pip 'setuptools==70.0.0' 'packaging>=24.1' 'wheel>=0.43' ninja

  echo '>>> Pre-installing imgui wheel'
  python -m pip install --only-binary=:all: \"imgui==2.0.0\" || python -m pip install \"imgui[glfw,opengl3]==2.0.0\"

  echo '>>> Installing Cython, OpenGL, and rosbridge dependencies'
  python -m pip install 'Cython>=0.24,<0.30' PyOpenGL roslibpy

  echo '>>> Installing thirdparty packages'
  python -m pip install --no-build-isolation -e thirdparty/mast3r/asmk
  python -m pip install --no-build-isolation -e thirdparty/mast3r
  python -m pip install --no-build-isolation thirdparty/in3d
  echo '>>> Installing faiss-cpu'
  python -m pip install --no-cache-dir 'faiss-cpu==1.7.4' || python -m pip install --no-cache-dir faiss-cpu

  echo '>>> Installing project itself'
  echo '>>> Ensuring compatible numpy for Python 3.12'
  python -m pip install --only-binary=:all: 'numpy>=1.26,<2'
  export PIP_ONLY_BINARY=":all:"
  export TORCH_CUDA_ARCH_LIST="11.0"
  export FORCE_CUDA=1
  python -m pip install --no-build-isolation . --no-cache-dir

  echo '>>> Final check of torch/cuda availability'
  python - <<'PY'
import torch, sys
print('torch:', torch.__version__)
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('GPU:', torch.cuda.get_device_name(0))
else:
    print('WARNING: torch.cuda.is_available() == False')
    print('Please confirm container has runtime: nvidia set')
PY

  echo '>>> Building Locobot ROS workspace'
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  cd /workspace/thor/ros2_ws
  colcon build --symlink-install

  echo '>>> Installation complete! Container is ready'
  mkdir -p \$(dirname \"\$SETUP_MARKER\")
  touch \"\$SETUP_MARKER\"
"
