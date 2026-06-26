#!/usr/bin/env bash
set -euo pipefail

# Workstation version of run_mast3r.sh for x86_64 NVIDIA GPUs.
# Uses an NVIDIA CUDA Ubuntu 22.04 base image with ROS 2 Humble.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Default parameters
DATASET_DEFAULT="datasets/tum/rgbd_dataset_freiburg1_desk"
CONFIG_DEFAULT="config/base.yaml"

DATASET="${1:-$DATASET_DEFAULT}"
CONFIG="${2:-$CONFIG_DEFAULT}"
EXTRA_ARGS="${3:-}"
IMAGE_NAME="mast3r-slam:workstation-humble"
LEGACY_IMAGE_NAME="mast3r-slam:workstation-stretch3-humble"
CONTAINER_NAME="mast3r_locobot_humble"
COMPOSE_FILE="docker/docker-compose.workstation_humble.yml"

# Set this to 1 after changing the Dockerfile and wanting a rebuild.
# Keep it at 0 to reuse the existing image for faster startup.
REBUILD_IMAGE=0

# Allow X11 for GUI
if command -v xhost > /dev/null 2>&1; then
  xhost +local:root > /dev/null 2>&1 || true
fi

# Build only when the image is missing, REBUILD_IMAGE=1, or FORCE_BUILD=1 is set.
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1 && docker image inspect "$LEGACY_IMAGE_NAME" > /dev/null 2>&1; then
  echo ">>> Reusing existing image: $LEGACY_IMAGE_NAME -> $IMAGE_NAME"
  docker tag "$LEGACY_IMAGE_NAME" "$IMAGE_NAME"
fi

if [ "${FORCE_BUILD:-$REBUILD_IMAGE}" = "1" ] || ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
  echo ">>> Building workstation image..."
  docker compose -f "$COMPOSE_FILE" build
else
  echo ">>> Reusing existing workstation image: $IMAGE_NAME"
fi

echo ">>> Starting workstation container..."
docker compose -f "$COMPOSE_FILE" up -d --no-build mast3r_locobot_humble

echo ">>> Setting up container environment..."
docker exec -it "$CONTAINER_NAME" env FORCE_SETUP="${FORCE_SETUP:-0}" bash -lc "
  set -euo pipefail

  SETUP_MARKER=/usr/local/share/mast3r_workstation_locobot_humble_setup_complete
  ROS_WS_SETUP_LINE='if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then source /workspace/thor/ros2_ws/install/setup.bash; fi'
  PS1_LINE='PS1=\"\[\e[1;34m\][locobot]\[\e[0m\] \w # \"'
  sed -i '/^PS1=.*\\[stretch3\\]/d; /^PS1=.*\\[locobot\\]/d' ~/.bashrc
  printf '%s\n' "\$PS1_LINE" >> ~/.bashrc
  if ! grep -Fq '/workspace/thor/ros2_ws/install/setup.bash' ~/.bashrc; then
    printf '%s\n' "\$ROS_WS_SETUP_LINE" >> ~/.bashrc
  fi

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
  python -m pip install -U pip 'setuptools==70.0.0' 'packaging>=24.1' 'wheel>=0.43' ninja backports.tarfile

  echo '>>> Ensuring PyTorch 2.7.1+cu128 for RTX 50-series'
  if ! python - <<'PY'
import sys
try:
    import torch
except Exception:
    sys.exit(1)
print('current torch:', torch.__version__)
print('current torch.version.cuda:', torch.version.cuda)
if not torch.__version__.startswith('2.7.1') or torch.version.cuda != '12.8':
    sys.exit(1)
PY
  then
    python -m pip install --force-reinstall --index-url https://download.pytorch.org/whl/cu128 \
      'torch==2.7.1' 'torchvision==0.22.1' 'torchaudio==2.7.1'
  fi

  echo '>>> Pre-installing imgui wheel'
  python -m pip install --only-binary=:all: \"imgui==2.0.0\" || python -m pip install \"imgui[glfw,opengl3]==2.0.0\"

  echo '>>> Installing Cython, OpenGL, and rosbridge dependencies'
  python -m pip install 'Cython>=0.24,<0.30' PyOpenGL roslibpy

  echo '>>> Generating vendored pyimgui Cython sources'
  cd /workspace/thor/MASt3R-SLAM/thirdparty/in3d/thirdparty/pyimgui
  python -m cython -I . -I imgui --cplus -o imgui/core.cpp imgui/core.pyx
  python -m cython -I . -I imgui --cplus -o imgui/internal.cpp imgui/internal.pyx
  ls -lh imgui/core.cpp imgui/core.h imgui/internal.cpp imgui/internal.h
  cd /workspace/thor/MASt3R-SLAM

  echo '>>> Installing thirdparty packages'
  python -m pip install --no-build-isolation --no-deps -e thirdparty/mast3r/asmk
  python -m pip install --no-build-isolation --no-deps -e thirdparty/mast3r
  python -m pip install --no-build-isolation --no-deps -e thirdparty/in3d
  echo '>>> Installing faiss-cpu'
  python -m pip install --no-cache-dir 'faiss-cpu==1.7.4' || python -m pip install --no-cache-dir faiss-cpu

  echo '>>> Installing lietorch without touching PyTorch'
  TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}" FORCE_CUDA=1 \
    python -m pip install --no-build-isolation --no-deps \
    'lietorch @ git+https://github.com/princeton-vl/lietorch.git'

  echo '>>> Installing project itself'
  echo '>>> Ensuring known-good numpy/opencv pins from workstation build'
  python -m pip install --only-binary=:all: 'numpy==1.26.4' 'opencv-python==4.8.1.78'
  export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"
  export FORCE_CUDA=1
  python -m pip install --no-build-isolation --no-deps . --no-cache-dir

  echo '>>> Final check of torch/cuda availability'
  python - <<'PY'
import torch, sys
print('torch:', torch.__version__)
print('torch.version.cuda:', torch.version.cuda)
print('CUDA available:', torch.cuda.is_available())
if torch.cuda.is_available():
    print('GPU:', torch.cuda.get_device_name(0))
else:
    print('WARNING: torch.cuda.is_available() == False')
    print('Please confirm container has runtime: nvidia set')
PY

  echo '>>> Building Locobot ROS workspace'
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd /workspace/thor/ros2_ws
  colcon build --symlink-install

  echo '>>> Installation complete! Container is ready'
  mkdir -p \$(dirname \"\$SETUP_MARKER\")
  touch \"\$SETUP_MARKER\"
"
