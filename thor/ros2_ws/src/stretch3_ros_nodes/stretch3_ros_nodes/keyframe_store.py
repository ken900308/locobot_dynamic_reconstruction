from typing import Dict, Iterable, List

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, Sim3Keyframe


class KeyframeStore:
    def __init__(self):
        self._items: Dict[KeyframeKey, Sim3Keyframe] = {}

    def upsert(self, keyframe: Sim3Keyframe) -> bool:
        is_new = keyframe.key not in self._items
        self._items[keyframe.key] = keyframe
        return is_new

    def get(self, key: KeyframeKey) -> Sim3Keyframe | None:
        return self._items.get(key)

    def by_other_robots(self, robot_id: str) -> Iterable[Sim3Keyframe]:
        for keyframe in self._items.values():
            if keyframe.robot_id != robot_id:
                yield keyframe

    def count(self) -> int:
        return len(self._items)

    def robot_counts(self) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for robot_id, _ in self._items:
            counts[robot_id] = counts.get(robot_id, 0) + 1
        return counts

    def all(self) -> List[Sim3Keyframe]:
        return list(self._items.values())
