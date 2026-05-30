from dataclasses import dataclass
from typing import Dict, Tuple

KeyframeKey = Tuple[str, int]


@dataclass(frozen=True)
class GlobalKeyframeRef:
    global_index: int
    robot_id: str
    kf_id: int
    keyframe_uid: str
    cache_path: str


class CrossRobotKeyframeIndex:
    def __init__(self):
        self._key_to_index: Dict[KeyframeKey, int] = {}
        self._records: list[GlobalKeyframeRef] = []

    def upsert(self, robot_id: str, kf_id: int, keyframe_uid: str, cache_path: str) -> GlobalKeyframeRef:
        key = (robot_id, int(kf_id))
        existing = self._key_to_index.get(key)
        if existing is not None:
            ref = GlobalKeyframeRef(existing, robot_id, int(kf_id), keyframe_uid, cache_path)
            self._records[existing] = ref
            return ref
        global_index = len(self._records)
        ref = GlobalKeyframeRef(global_index, robot_id, int(kf_id), keyframe_uid, cache_path)
        self._key_to_index[key] = global_index
        self._records.append(ref)
        return ref

    def __len__(self) -> int:
        return len(self._records)

    def robot_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for record in self._records:
            counts[record.robot_id] = counts.get(record.robot_id, 0) + 1
        return counts

    def records(self) -> list[GlobalKeyframeRef]:
        return list(self._records)
