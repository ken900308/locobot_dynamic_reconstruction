import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import torch
except Exception:  # pragma: no cover - ROS package can still import without torch in tooling.
    torch = None


@dataclass(frozen=True)
class NativeKeyframeManifest:
    robot_id: str
    kf_id: int
    keyframe_uid: str
    cache_path: str
    schema: str
    tensor_shapes: dict[str, list[int]]


@dataclass(frozen=True)
class NativeKeyframeCacheRecord:
    manifest: NativeKeyframeManifest
    payload: dict[str, Any]


def manifest_from_metadata_json(data: str) -> NativeKeyframeManifest:
    payload = json.loads(data)
    cache_path = str(payload.get("native_cache_path", ""))
    if not cache_path:
        raise ValueError("metadata has no native_cache_path")
    robot_id = str(payload["robot_id"])
    kf_id = int(payload["kf_id"])
    return NativeKeyframeManifest(
        robot_id=robot_id,
        kf_id=kf_id,
        keyframe_uid=str(payload.get("keyframe_uid", f"{robot_id}_kf_{kf_id:06d}")),
        cache_path=cache_path,
        schema=str(payload.get("native_cache_schema", "")),
        tensor_shapes=dict(payload.get("native_cache_tensors", {})),
    )


def validate_native_cache_manifest(manifest: NativeKeyframeManifest) -> None:
    path = Path(manifest.cache_path)
    if not path.exists():
        raise FileNotFoundError(f"native keyframe cache does not exist: {path}")
    if not path.is_file():
        raise ValueError(f"native keyframe cache path is not a file: {path}")


def load_native_keyframe_cache(manifest: NativeKeyframeManifest, map_location: str = "cpu") -> NativeKeyframeCacheRecord:
    if torch is None:
        raise RuntimeError("torch is required to load native keyframe cache files")
    validate_native_cache_manifest(manifest)
    payload = torch.load(manifest.cache_path, map_location=map_location, weights_only=False)
    if payload.get("schema") != manifest.schema:
        raise ValueError(
            f"native cache schema mismatch for {manifest.keyframe_uid}: "
            f"metadata={manifest.schema!r}, file={payload.get('schema')!r}"
        )
    if payload.get("robot_id") != manifest.robot_id or int(payload.get("kf_id", -1)) != manifest.kf_id:
        raise ValueError(f"native cache key mismatch for {manifest.keyframe_uid}")
    return NativeKeyframeCacheRecord(manifest=manifest, payload=payload)
