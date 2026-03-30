#!/usr/bin/env bash
set -euo pipefail

echo "=========================================="
echo "   Thor GPU Environment Check"
echo "=========================================="

echo "[1/8] OS & Kernel"
cat /etc/os-release | grep -E "PRETTY_NAME|VERSION_ID"
echo "Kernel: $(uname -r)"

echo "[2/8] Python Version"
python3 --version

echo "[3/8] CUDA & NVCC"
if command -v nvcc >/dev/null 2>&1; then
  nvcc --version | grep -E "release|Build"
  echo "CUDA_HOME: ${CUDA_HOME:-not set}"
else
  echo "❌ nvcc not found"
fi

echo "[4/8] PyTorch & CUDA Availability"
python3 - <<'PY'
import torch
print(f"PyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA version: {torch.version.cuda}")
    print(f"GPU count: {torch.cuda.device_count()}")
    print(f"GPU 0: {torch.cuda.get_device_name(0)}")
    cap = torch.cuda.get_device_capability(0)
    print(f"Compute capability: {cap[0]}.{cap[1]} (SM {cap[0]}{cap[1]})")
    print(f"Arch list: {torch.cuda.get_arch_list()}")
else:
    print("⚠️ CUDA not available")
PY

echo "[5/8] Key Python Packages"
python3 - <<'PY'
packages = [
    ("numpy", "numpy"),
    ("opencv", "cv2"),
    ("open3d", "open3d"),
    ("matplotlib", "matplotlib"),
    ("scipy", "scipy"),
    ("Pillow", "PIL"),
    ("einops", "einops"),
    ("torch", "torch")
]
for name, module in packages:
    try:
        mod = __import__(module)
        ver = getattr(mod, "__version__", "unknown")
        print(f"  ✓ {name}: {ver}")
    except ImportError:
        print(f"  ✗ {name}: NOT INSTALLED")
PY

echo "[6/8] MASt3R-SLAM"
if [ -d /workspace/thor/MASt3R-SLAM ]; then
  echo "  ✓ /workspace/thor/MASt3R-SLAM exists"
  python3 - <<'PY'
import sys
sys.path.insert(0, '/workspace/thor/MASt3R-SLAM')
try:
    import mast3r_slam
    print("  ✓ mast3r_slam importable")
except ImportError as e:
    print(f"  ✗ mast3r_slam import failed: {e}")
PY
else
  echo "  ✗ /workspace/MASt3R-SLAM not found"
fi

echo "[7/8] NVIDIA Environment"
echo "NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES:-not set}"
echo "NVIDIA_DRIVER_CAPABILITIES: ${NVIDIA_DRIVER_CAPABILITIES:-not set}"

echo "[8/8] GPU Memory"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,memory.total,memory.used,memory.free --format=csv,noheader
else
  echo "nvidia-smi not available"
fi

echo "=========================================="
echo "✅ GPU Environment Check Complete"
echo "=========================================="
