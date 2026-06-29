#!/usr/bin/env bash

set -euo pipefail

MAST3R_ROOT="${MAST3R_ROOT:-/workspace/thor/MASt3R-SLAM}"
TORCH_SITE_PACKAGES="/usr/local/lib/python3.10/dist-packages"

if [[ ! -d "${MAST3R_ROOT}" ]]; then
    echo "ERROR: MASt3R-SLAM directory not found: ${MAST3R_ROOT}" >&2
    exit 1
fi

echo "[1/6] Reinstalling PyTorch 2.7.1+cu128"
python3 -m pip uninstall -y torch || true
rm -rf \
    "${TORCH_SITE_PACKAGES}/torch" \
    "${TORCH_SITE_PACKAGES}/torch-2.7.1.dist-info" \
    "${TORCH_SITE_PACKAGES}/torch-2.7.1+cu128.dist-info"
python3 -m pip install --no-cache-dir --no-deps \
    --index-url https://download.pytorch.org/whl/cu128 \
    torch==2.7.1+cu128

echo "[2/6] Installing Hugging Face Hub"
python3 -m pip install --no-cache-dir 'huggingface-hub>=0.22'

echo "[3/6] Rebuilding ASMK extension"
cd "${MAST3R_ROOT}"
python3 -m pip install \
    --force-reinstall \
    --no-deps \
    --no-build-isolation \
    -e thirdparty/mast3r/asmk

echo "[4/6] Rebuilding cuRoPE2D CUDA extension"
cd "${MAST3R_ROOT}/thirdparty/mast3r/dust3r/croco/models/curope"
TORCH_CUDA_ARCH_LIST=12.0 FORCE_CUDA=1 \
python3 -m pip install \
    --no-build-isolation \
    --no-deps \
    -e .

echo "[5/6] Installing reconstruction runtime packages"
python3 -m pip install --no-cache-dir \
    'moderngl==5.12.0' \
    'moderngl-window==2.4.6' \
    'pyglet>=2.0' \
    pyglm \
    roma \
    trimesh

echo "[6/6] Verifying reconstruction environment"
python3 - <<'PY'
import curope
import torch
from asmk import hamming

assert torch.__version__ == "2.7.1+cu128", torch.__version__
assert torch.version.cuda == "12.8", torch.version.cuda
assert torch.cuda.is_available(), "CUDA is not available"
print("PyTorch:", torch.__version__)
print("CUDA:", torch.version.cuda)
print("GPU:", torch.cuda.get_device_name(0))
print("ASMK hamming:", hamming.__file__)
print("cuRoPE2D:", curope.__file__)
PY

CHECK_SCRIPT="$(dirname "${MAST3R_ROOT}")/check_recon_env.sh"
if [[ -x "${CHECK_SCRIPT}" ]]; then
    "${CHECK_SCRIPT}"
fi

echo "Reconstruction environment repair completed."
