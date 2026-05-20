import re
from dataclasses import dataclass
from typing import Dict

from sensor_msgs.msg import Image

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey

_KEYFRAME_RE = re.compile(r"^(?P<robot>.+)_kf_(?P<kf_id>\d+)$")


@dataclass(frozen=True)
class ImageRecord:
    key: KeyframeKey
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    width: int
    height: int
    encoding: str
    msg: Image


class KeyframeImageStore:
    def __init__(self):
        self._items: Dict[KeyframeKey, ImageRecord] = {}

    def upsert(self, msg: Image) -> ImageRecord:
        key = parse_keyframe_frame_id(msg.header.frame_id)
        record = ImageRecord(
            key=key,
            frame_id=msg.header.frame_id,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            width=int(msg.width),
            height=int(msg.height),
            encoding=str(msg.encoding),
            msg=msg,
        )
        self._items[key] = record
        return record

    def get(self, key: KeyframeKey) -> ImageRecord | None:
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
