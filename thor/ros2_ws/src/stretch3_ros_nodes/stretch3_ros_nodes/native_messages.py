from __future__ import annotations

import json
from pathlib import Path
from typing import Any

try:
    import torch
except Exception:  # pragma: no cover
    torch = None


EDGE_SCHEMA = "multi_robot_native_dense_factor_edge_v1"
POSE_SCHEMA = "multi_robot_native_optimized_keyframe_pose_v1"


def dumps(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"))


def loads(text: str) -> dict[str, Any]:
    return json.loads(text)


def save_edge_cache(cache_dir: str, edge_uid: str, tensors: dict[str, Any], metadata: dict[str, Any]) -> str:
    if torch is None:
        raise RuntimeError("torch is required to save native edge caches")
    path_dir = Path(cache_dir).expanduser()
    path_dir.mkdir(parents=True, exist_ok=True)
    path = path_dir / f"{edge_uid}.pt"
    tmp = path_dir / f".{edge_uid}.tmp.pt"
    torch.save({"schema": EDGE_SCHEMA, "metadata": metadata, "tensors": tensors}, tmp)
    tmp.replace(path)
    return str(path)


def load_edge_cache(path: str) -> dict[str, Any]:
    if torch is None:
        raise RuntimeError("torch is required to load native edge caches")
    payload = torch.load(path, map_location="cpu", weights_only=False)
    if payload.get("schema") != EDGE_SCHEMA:
        raise ValueError(f"unsupported edge cache schema: {payload.get('schema')}")
    return payload


def tensor_shape(value) -> list[int]:
    return list(value.shape) if hasattr(value, "shape") else []
