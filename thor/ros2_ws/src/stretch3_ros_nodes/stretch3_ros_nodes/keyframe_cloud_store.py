import re
from dataclasses import dataclass
from typing import Dict, Tuple

from sensor_msgs.msg import PointCloud2

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey

_KEYFRAME_RE = re.compile(r"^(?P<robot>.+)_kf_(?P<kf_id>\d+)$")


@dataclass(frozen=True)
class CloudRecord:
    key: KeyframeKey
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    point_count: int
    point_step: int
    fields: tuple[str, ...]
    msg: PointCloud2


class KeyframeCloudStore:
    def __init__(self):
        self._items: Dict[KeyframeKey, CloudRecord] = {}

    def upsert(self, msg: PointCloud2) -> CloudRecord:
        key = parse_keyframe_frame_id(msg.header.frame_id)
        record = CloudRecord(
            key=key,
            frame_id=msg.header.frame_id,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            point_count=int(msg.width) * int(msg.height),
            point_step=int(msg.point_step),
            fields=tuple(field.name for field in msg.fields),
            msg=msg,
        )
        self._items[key] = record
        return record

    def get(self, key: KeyframeKey) -> CloudRecord | None:
        return self._items.get(key)

    def has(self, key: KeyframeKey) -> bool:
        return key in self._items

    def count(self) -> int:
        return len(self._items)


def parse_keyframe_frame_id(frame_id: str) -> KeyframeKey:
    match = _KEYFRAME_RE.match(frame_id or "")
    if match is None:
        raise ValueError(f"expected frame_id like robot1_kf_000123, got {frame_id!r}")
    return (match.group("robot"), int(match.group("kf_id")))
