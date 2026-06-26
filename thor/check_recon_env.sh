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
echo "   MASt3R-SLAM Reconstruction Env Check"
echo "=========================================="

echo "[1/8] Project Root"
MAST3R_ROOT=""
for candidate in /workspace/thor/MASt3R-SLAM /workspace/MASt3R-SLAM "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/MASt3R-SLAM"; do
  if [ -d "$candidate" ]; then
    MAST3R_ROOT="$candidate"
    break
  fi
done

if [ -z "$MAST3R_ROOT" ]; then
  fail "MASt3R-SLAM root not found"
else
  ok "MASt3R-SLAM root: $MAST3R_ROOT"
fi

echo "[2/8] ROS 2 Environment"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [ -f "$ROS_SETUP" ]; then
  set +u
  source "$ROS_SETUP"
  set -u
  ok "Sourced $ROS_SETUP"
else
  fail "$ROS_SETUP not found"
fi

echo "[3/8] Python Build Tools"
python3 - <<'PY' || FAILED=1
import sys
mods = [
    ("setuptools", "setuptools"),
    ("setuptools.build_meta", "setuptools.build_meta"),
    ("wheel", "wheel"),
    ("packaging", "packaging"),
    ("ninja", "ninja"),
    ("backports.tarfile", "backports.tarfile"),
]
failed = False
for name, module in mods:
    try:
        mod = __import__(module, fromlist=["*"])
        print(f"  ✓ {name}: {getattr(mod, '__version__', 'importable')}")
    except Exception as exc:
        failed = True
        print(f"  ✗ {name}: {exc}")
if failed:
    sys.exit(1)
PY

echo "[4/8] CUDA / PyTorch"
python3 - <<'PY' || FAILED=1
import sys
import traceback

try:
    import torch
except Exception:
    traceback.print_exc()
    sys.exit(1)

print(f"  torch: {torch.__version__}")
print(f"  torch.version.cuda: {torch.version.cuda}")
if not torch.__version__.startswith("2.7.1"):
    print("  ✗ expected torch 2.7.1")
    sys.exit(1)
if torch.version.cuda != "12.8":
    print("  ✗ expected torch.version.cuda == 12.8")
    sys.exit(1)
try:
    torch.cuda.init()
    print(f"  ✓ CUDA init OK: {torch.cuda.get_device_name(0)}")
    print(f"  ✓ compute capability: {torch.cuda.get_device_capability(0)}")
except Exception:
    traceback.print_exc()
    sys.exit(1)
PY

echo "[5/8] Reconstruction Python Modules"
python3 - "$MAST3R_ROOT" <<'PY' || FAILED=1
import sys
root = sys.argv[1]
sys.path.insert(0, root)

mods = [
    ("numpy", "numpy"),
    ("cv2", "cv2"),
    ("torch", "torch"),
    ("yaml", "yaml"),
    ("tqdm", "tqdm"),
    ("PIL", "PIL"),
    ("einops", "einops"),
    ("lietorch", "lietorch"),
    ("mast3r_slam", "mast3r_slam"),
    ("mast3r_slam_backends", "mast3r_slam_backends"),
    ("mast3r_slam.matching", "mast3r_slam.matching"),
    ("mast3r_slam.global_opt", "mast3r_slam.global_opt"),
    ("mast3r_slam.frame", "mast3r_slam.frame"),
    ("mast3r_slam.tracker", "mast3r_slam.tracker"),
    ("main_mast3r", "main_mast3r"),
    ("mast3r", "mast3r"),
    ("mast3r.model", "mast3r.model"),
    ("dust3r", "dust3r"),
    ("asmk", "asmk"),
    ("plyfile", "plyfile"),
    ("natsort", "natsort"),
    ("evo", "evo"),
]
failed = False
for name, module in mods:
    try:
        mod = __import__(module, fromlist=["*"])
        print(f"  ✓ {name}: {getattr(mod, '__file__', 'built-in')}")
    except Exception as exc:
        failed = True
        print(f"  ✗ {name}: {exc}")
if failed:
    sys.exit(1)
PY

echo "[6/8] Native Extension Symbols"
python3 - <<'PY' || FAILED=1
import sys
try:
    import torch  # Load torch shared libraries such as libc10 before the extension.
    import mast3r_slam_backends as backend
except Exception as exc:
    print(f"  ✗ mast3r_slam_backends import failed: {exc}")
    sys.exit(1)

required = [
    "gauss_newton_points",
    "gauss_newton_rays",
    "gauss_newton_calib",
    "iter_proj",
    "refine_matches",
]
missing = [name for name in required if not hasattr(backend, name)]
if missing:
    print(f"  ✗ missing backend functions: {missing}")
    sys.exit(1)
print(f"  ✓ mast3r_slam_backends: {backend.__file__}")
for name in required:
    print(f"  ✓ {name}")
PY

echo "[7/8] ROS / Robot I/O Modules"
python3 - <<'PY' || FAILED=1
import sys
mods = [
    ("rclpy", "rclpy"),
    ("sensor_msgs.msg", "sensor_msgs.msg"),
    ("geometry_msgs.msg", "geometry_msgs.msg"),
    ("std_msgs.msg", "std_msgs.msg"),
    ("cv_bridge", "cv_bridge"),
    ("roslibpy", "roslibpy"),
]
failed = False
for name, module in mods:
    try:
        mod = __import__(module, fromlist=["*"])
        print(f"  ✓ {name}: {getattr(mod, '__file__', 'importable')}")
    except Exception as exc:
        failed = True
        print(f"  ✗ {name}: {exc}")
if failed:
    sys.exit(1)
PY

echo "[8/8] Runtime Assets"
if [ -n "$MAST3R_ROOT" ]; then
  for path in \
    "$MAST3R_ROOT/config/base.yaml" \
    "$MAST3R_ROOT/config/calib.yaml" \
    "$MAST3R_ROOT/config/intrinsics.yaml" \
    "$MAST3R_ROOT/checkpoints/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth"; do
    if [ -e "$path" ]; then
      ok "$path"
    else
      fail "$path missing"
    fi
  done
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader || warn "nvidia-smi query failed"
else
  warn "nvidia-smi not available"
fi

if [ "$FAILED" -ne 0 ]; then
  echo "=========================================="
  echo "❌ Reconstruction Env Check Failed"
  echo "=========================================="
  exit 1
fi

if [ "$WARNED" -ne 0 ]; then
  echo "=========================================="
  echo "⚠️ Reconstruction Env Check Passed With Warnings"
  echo "=========================================="
else
  echo "=========================================="
  echo "✅ Reconstruction Env Check Passed"
  echo "=========================================="
fi
