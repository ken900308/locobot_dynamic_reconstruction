import os
from pathlib import Path
from typing import Any, Dict, Optional

import torch


SCHEMA = "mast3r_native_keyframe_cache_v1"


def _as_cpu_tensor(value: Any) -> Optional[torch.Tensor]:
    if value is None:
        return None
    if isinstance(value, torch.Tensor):
        return value.detach().cpu()
    try:
        return torch.as_tensor(value).detach().cpu()
    except Exception:
        return None


def _tensor_shape(value: Any) -> list[int]:
    tensor = _as_cpu_tensor(value)
    return list(tensor.shape) if tensor is not None else []


class NativeKeyframeCacheWriter:
    """Persist native MASt3R-SLAM keyframe tensors for a central cross-robot backend."""

    def __init__(self, robot_id: str, cache_dir: str | None = None):
        self.robot_id = robot_id
        requested_dir = cache_dir or os.environ.get("MAST3R_NATIVE_KEYFRAME_CACHE_DIR", "")
        self.cache_dir = Path(requested_dir).expanduser() if requested_dir else None
        self.enabled = self.cache_dir is not None
        if self.cache_dir is not None:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

    def write(self, keyframe: Any) -> Optional[Dict[str, Any]]:
        if not self.enabled or self.cache_dir is None:
            return None
        if getattr(keyframe, "X_canon", None) is None or getattr(keyframe, "C", None) is None:
            return None

        kf_id = int(keyframe.frame_id)
        keyframe_uid = f"{self.robot_id}_kf_{kf_id:06d}"
        path = self.cache_dir / f"{keyframe_uid}.pt"
        tmp_path = self.cache_dir / f".{keyframe_uid}.tmp.pt"

        payload = {
            "schema": SCHEMA,
            "robot_id": self.robot_id,
            "kf_id": kf_id,
            "keyframe_uid": keyframe_uid,
            "sim3_data": _as_cpu_tensor(keyframe.T_WC.data),
            "img": _as_cpu_tensor(getattr(keyframe, "img", None)),
            "img_shape": _as_cpu_tensor(getattr(keyframe, "img_shape", None)),
            "img_true_shape": _as_cpu_tensor(getattr(keyframe, "img_true_shape", None)),
            "uimg": _as_cpu_tensor(getattr(keyframe, "uimg", None)),
            "X_canon": _as_cpu_tensor(getattr(keyframe, "X_canon", None)),
            "C": _as_cpu_tensor(getattr(keyframe, "C", None)),
            "feat": _as_cpu_tensor(getattr(keyframe, "feat", None)),
            "pos": _as_cpu_tensor(getattr(keyframe, "pos", None)),
            "N": int(getattr(keyframe, "N", 1) or 1),
            "N_updates": int(getattr(keyframe, "N_updates", 0) or 0),
            "K": _as_cpu_tensor(getattr(keyframe, "K", None)),
        }
        torch.save(payload, tmp_path)
        tmp_path.replace(path)

        return {
            "native_cache_schema": SCHEMA,
            "native_cache_path": str(path),
            "native_cache_device": "cpu",
            "native_cache_tensors": {
                "T_WC": _tensor_shape(payload["sim3_data"]),
                "img": _tensor_shape(payload["img"]),
                "uimg": _tensor_shape(payload["uimg"]),
                "X_canon": _tensor_shape(payload["X_canon"]),
                "C": _tensor_shape(payload["C"]),
                "feat": _tensor_shape(payload["feat"]),
                "pos": _tensor_shape(payload["pos"]),
            },
        }
