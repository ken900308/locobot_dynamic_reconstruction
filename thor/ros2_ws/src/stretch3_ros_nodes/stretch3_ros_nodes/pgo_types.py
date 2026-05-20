import json
from dataclasses import dataclass
from typing import Any, Dict

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, key_to_uid


@dataclass(frozen=True)
class PoseConstraint:
    from_key: KeyframeKey
    to_key: KeyframeKey
    from_uid: str
    to_uid: str
    relative_sim3_data: list[float]
    confidence: float
    rmse_m: float
    inlier_count: int
    match_count: int
    payload: Dict[str, Any]

    @property
    def key(self) -> tuple[KeyframeKey, KeyframeKey]:
        return (self.from_key, self.to_key)


def parse_pose_constraint(text: str) -> PoseConstraint:
    data: Dict[str, Any] = json.loads(text)
    if data.get("schema") != "inter_robot_sim3_constraint_v1":
        raise ValueError(f"unsupported pose constraint schema: {data.get('schema')}")
    relative = [float(x) for x in data.get("relative_sim3_data", [])]
    if len(relative) != 8:
        raise ValueError("pose constraint requires 8-value relative_sim3_data")

    from_key = (str(data["from_robot"]), int(data["from_kf_id"]))
    to_key = (str(data["to_robot"]), int(data["to_kf_id"]))
    return PoseConstraint(
        from_key=from_key,
        to_key=to_key,
        from_uid=str(data.get("from_uid", key_to_uid(from_key))),
        to_uid=str(data.get("to_uid", key_to_uid(to_key))),
        relative_sim3_data=relative,
        confidence=float(data.get("confidence", 1.0)),
        rmse_m=float(data.get("rmse_m", 0.0)),
        inlier_count=int(data.get("inlier_count", 0)),
        match_count=int(data.get("match_count", 0)),
        payload=data,
    )
