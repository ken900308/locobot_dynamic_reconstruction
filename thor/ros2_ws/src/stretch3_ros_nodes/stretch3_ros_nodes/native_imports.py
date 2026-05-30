from __future__ import annotations

import os
import sys
from pathlib import Path


def configure_mast3r_imports(mast3r_slam_root: str) -> Path:
    root = Path(mast3r_slam_root).expanduser().resolve()
    thirdparty = root / "thirdparty" / "mast3r"
    for path in (str(root), str(thirdparty)):
        if path not in sys.path:
            sys.path.insert(0, path)
    return root


def default_cache_root() -> str:
    return os.environ.get("MAST3R_NATIVE_BACKEND_CACHE_DIR", "/workspace/shared_native_keyframe_cache/backend")
