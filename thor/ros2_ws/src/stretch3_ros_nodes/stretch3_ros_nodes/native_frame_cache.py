from __future__ import annotations

from dataclasses import dataclass
from typing import Any

try:
    import torch
except Exception:  # pragma: no cover
    torch = None

from stretch3_ros_nodes.cross_robot_native_cache import NativeKeyframeManifest, load_native_keyframe_cache


@dataclass(frozen=True)
class NativeFrameRecord:
    robot_id: str
    kf_id: int
    keyframe_uid: str
    global_index: int
    cache_path: str
    payload: dict[str, Any]


def _to_device_tensor(value, device: str):
    if value is None:
        return None
    if torch is None:
        raise RuntimeError("torch is required")
    return value.to(device=device) if hasattr(value, "to") else torch.as_tensor(value, device=device)


def load_frame_record(manifest: NativeKeyframeManifest, global_index: int, device: str = "cuda:0") -> NativeFrameRecord:
    record = load_native_keyframe_cache(manifest, map_location="cpu")
    payload = dict(record.payload)
    for key in ("sim3_data", "img", "img_shape", "img_true_shape", "X_canon", "C", "feat", "pos", "K"):
        payload[key] = _to_device_tensor(payload.get(key), device)
    # uimg is kept on CPU because native Frame stores colors on CPU.
    return NativeFrameRecord(
        robot_id=manifest.robot_id,
        kf_id=manifest.kf_id,
        keyframe_uid=manifest.keyframe_uid,
        global_index=global_index,
        cache_path=manifest.cache_path,
        payload=payload,
    )


def record_to_frame(record: NativeFrameRecord):
    import lietorch
    from mast3r_slam.frame import Frame

    p = record.payload
    frame = Frame(
        int(record.kf_id),
        p["img"],
        p["img_shape"],
        p["img_true_shape"],
        p.get("uimg"),
        lietorch.Sim3(p["sim3_data"]),
    )
    frame.X_canon = p["X_canon"]
    frame.C = p["C"]
    frame.feat = p["feat"]
    frame.pos = p["pos"]
    frame.N = int(p.get("N", 1) or 1)
    frame.N_updates = int(p.get("N_updates", 0) or 0)
    frame.K = p.get("K")
    return frame


class NativeFrameCollection:
    def __init__(self, device: str):
        self.device = device
        self._frames = {}
        self._records = {}
        self.K = None

    def upsert(self, record: NativeFrameRecord) -> None:
        self._records[int(record.global_index)] = record
        self._frames[int(record.global_index)] = record_to_frame(record)
        if self.K is None and record.payload.get("K") is not None:
            self.K = record.payload.get("K")

    def has(self, global_index: int) -> bool:
        return int(global_index) in self._frames

    def __getitem__(self, idx: int):
        return self._frames[int(idx)]

    def __len__(self) -> int:
        return len(self._frames)

    def update_T_WCs(self, T_WCs, idx) -> None:
        for local_i, global_idx in enumerate(idx.detach().cpu().tolist()):
            self._frames[int(global_idx)].T_WC = T_WCs[local_i]

    def record(self, global_index: int) -> NativeFrameRecord:
        return self._records[int(global_index)]

    def records(self) -> list[NativeFrameRecord]:
        return [self._records[k] for k in sorted(self._records)]
