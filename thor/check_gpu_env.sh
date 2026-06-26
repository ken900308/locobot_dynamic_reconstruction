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
echo "   MASt3R-SLAM GPU Environment Check"
echo "=========================================="

echo "[1/9] OS & Kernel"
grep -E "PRETTY_NAME|VERSION_ID" /etc/os-release || true
echo "Kernel: $(uname -r)"

echo "[2/9] Python Version"
python3 --version || fail "python3 not found"

echo "[3/9] CUDA Toolkit"
if command -v nvcc >/dev/null 2>&1; then
  nvcc --version | grep -E "release|Build" || nvcc --version
  echo "CUDA_HOME: ${CUDA_HOME:-not set}"
else
  warn "nvcc not found"
fi

echo "[4/9] NVIDIA Runtime & Devices"
echo "NVIDIA_VISIBLE_DEVICES: ${NVIDIA_VISIBLE_DEVICES:-not set}"
echo "NVIDIA_DRIVER_CAPABILITIES: ${NVIDIA_DRIVER_CAPABILITIES:-not set}"
if compgen -G "/dev/nvidia*" >/dev/null; then
  ls -l /dev/nvidia* || true
else
  fail "No /dev/nvidia* devices visible inside container"
fi
if [ ! -e /dev/nvidia-uvm ]; then
  warn "/dev/nvidia-uvm not found; PyTorch CUDA init may fail"
fi

echo "[5/9] NVIDIA-SMI"
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used,memory.free --format=csv,noheader || fail "nvidia-smi failed"
else
  fail "nvidia-smi not available"
fi

echo "[6/9] PyTorch & CUDA Availability"
python3 - <<'PY' || FAILED=1
import sys
import traceback

try:
    import torch
except Exception:
    traceback.print_exc()
    sys.exit(1)

print(f"PyTorch: {torch.__version__}")
print(f"torch.version.cuda: {torch.version.cuda}")
print(f"CUDA available: {torch.cuda.is_available()}")

try:
    torch.cuda.init()
    print("CUDA init: OK")
    print(f"GPU count: {torch.cuda.device_count()}")
    if torch.cuda.device_count():
        print(f"GPU 0: {torch.cuda.get_device_name(0)}")
        cap = torch.cuda.get_device_capability(0)
        print(f"Compute capability: {cap[0]}.{cap[1]} (SM {cap[0]}{cap[1]})")
        print(f"Arch list: {torch.cuda.get_arch_list()}")
except Exception:
    print("CUDA init: FAILED")
    traceback.print_exc()
    sys.exit(1)
PY

echo "[7/9] Key Python Packages"
python3 - <<'PY' || FAILED=1
packages = [
    ("numpy", "numpy"),
    ("opencv", "cv2"),
    ("open3d", "open3d"),
    ("matplotlib", "matplotlib"),
    ("scipy", "scipy"),
    ("Pillow", "PIL"),
    ("einops", "einops"),
    ("torch", "torch"),
    ("lietorch", "lietorch"),
    ("imgui", "imgui"),
    ("in3d", "in3d"),
]
missing = False
for name, module in packages:
    try:
        mod = __import__(module)
        ver = getattr(mod, "__version__", "unknown")
        path = getattr(mod, "__file__", "unknown")
        print(f"  ✓ {name}: {ver} ({path})")
    except Exception as exc:
        missing = True
        print(f"  ✗ {name}: {exc}")
if missing:
    raise SystemExit(1)
PY

echo "[8/9] MASt3R-SLAM Path"
MAST3R_ROOT=""
for candidate in /workspace/thor/MASt3R-SLAM /workspace/MASt3R-SLAM; do
  if [ -d "$candidate" ]; then
    MAST3R_ROOT="$candidate"
    break
  fi
done

if [ -n "$MAST3R_ROOT" ]; then
  ok "$MAST3R_ROOT exists"
  python3 - "$MAST3R_ROOT" <<'PY' || FAILED=1
import sys
root = sys.argv[1]
sys.path.insert(0, root)
try:
    import mast3r_slam
    print(f"  ✓ mast3r_slam importable: {mast3r_slam.__file__}")
except Exception as exc:
    print(f"  ✗ mast3r_slam import failed: {exc}")
    raise SystemExit(1)
PY
else
  fail "MASt3R-SLAM root not found under /workspace/thor or /workspace"
fi

echo "[9/9] Summary"
if [ "$FAILED" -ne 0 ]; then
  echo "=========================================="
  echo "❌ GPU Environment Check Failed"
  echo "=========================================="
  exit 1
fi

if [ "$WARNED" -ne 0 ]; then
  echo "=========================================="
  echo "⚠️ GPU Environment Check Passed With Warnings"
  echo "=========================================="
else
  echo "=========================================="
  echo "✅ GPU Environment Check Passed"
  echo "=========================================="
fi
