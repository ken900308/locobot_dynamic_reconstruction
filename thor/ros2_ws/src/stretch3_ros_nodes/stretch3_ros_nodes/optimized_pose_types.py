import json
from dataclasses import dataclass
from typing import Any, Dict

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, key_to_uid
from stretch3_ros_nodes.sim3_math import Sim3


@dataclass(frozen=True)
class OptimizedKeyframePose:
    key: KeyframeKey
    keyframe_uid: str
    anchor_robot: str
    transform: Sim3
    payload: Dict[str, Any]


def parse_optimized_pose(text: str) -> OptimizedKeyframePose:
    data: Dict[str, Any] = json.loads(text)
    if data.get("schema") != "multi_robot_optimized_keyframe_pose_v1":
        raise ValueError(f"unsupported optimized pose schema: {data.get('schema')}")
    key = (str(data["robot_id"]), int(data["kf_id"]))
    sim3_data = [float(x) for x in data.get("optimized_sim3_data", [])]
    return OptimizedKeyframePose(
        key=key,
        keyframe_uid=str(data.get("keyframe_uid", key_to_uid(key))),
        anchor_robot=str(data.get("anchor_robot", "")),
        transform=Sim3.from_list(sim3_data),
        payload=data,
    )
