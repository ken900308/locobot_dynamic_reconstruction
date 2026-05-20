import json
from typing import Any, Dict

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, key_to_uid


def make_keyframe_cloud_summary(
    key: KeyframeKey,
    revision: int,
    point_count: int,
    output_frame: str,
    voxel_leaf_size: float,
    source: str,
) -> str:
    payload: Dict[str, Any] = {
        "schema": "multi_robot_optimized_keyframe_cloud_summary_v1",
        "event": "replace",
        "robot_id": key[0],
        "kf_id": key[1],
        "keyframe_uid": key_to_uid(key),
        "revision": int(revision),
        "point_count": int(point_count),
        "frame_id": output_frame,
        "voxel_leaf_size": float(voxel_leaf_size),
        "source": source,
    }
    return json.dumps(payload, separators=(",", ":"))


def make_optimized_map_summary(
    revision: int,
    keyframe_count: int,
    point_count: int,
    output_frame: str,
    max_merged_points: int,
    voxel_leaf_size: float,
) -> str:
    payload: Dict[str, Any] = {
        "schema": "multi_robot_optimized_map_summary_v1",
        "revision": int(revision),
        "keyframe_count": int(keyframe_count),
        "point_count": int(point_count),
        "frame_id": output_frame,
        "max_merged_points": int(max_merged_points),
        "voxel_leaf_size": float(voxel_leaf_size),
    }
    return json.dumps(payload, separators=(",", ":"))
