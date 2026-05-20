import json

from stretch3_ros_nodes.pgo_solver import PgoSolution
from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, key_to_uid
from stretch3_ros_nodes.sim3_math import Sim3


def make_robot_alignments_json(solution: PgoSolution) -> str:
    payload = {
        "schema": "multi_robot_sim3_alignments_v1",
        "anchor_robot": solution.anchor_robot,
        "received_constraint_count": solution.received_constraint_count,
        "constraint_count": solution.constraint_count,
        "used_constraint_count": solution.used_constraint_count,
        "rejected_constraint_count": solution.rejected_constraint_count,
        "rejection_counts": solution.rejection_counts,
        "pair_observation_counts": solution.pair_observation_counts,
        "pair_used_counts": solution.pair_used_counts,
        "sim3_layout": "tx_ty_tz_qx_qy_qz_qw_scale",
        "robot_alignments": {
            robot_id: transform.to_list()
            for robot_id, transform in sorted(solution.robot_alignments.items())
        },
    }
    return json.dumps(payload, separators=(",", ":"))


def make_optimized_pose_json(key: KeyframeKey, transform: Sim3, solution: PgoSolution) -> str:
    payload = {
        "schema": "multi_robot_optimized_keyframe_pose_v1",
        "robot_id": key[0],
        "kf_id": key[1],
        "keyframe_uid": key_to_uid(key),
        "anchor_robot": solution.anchor_robot,
        "optimized_sim3_data": transform.to_list(),
        "sim3_layout": "tx_ty_tz_qx_qy_qz_qw_scale",
        "received_constraint_count": solution.received_constraint_count,
        "constraint_count": solution.constraint_count,
        "used_constraint_count": solution.used_constraint_count,
        "rejected_constraint_count": solution.rejected_constraint_count,
    }
    return json.dumps(payload, separators=(",", ":"))


def make_solution_summary(solution: PgoSolution) -> str:
    aligned = ",".join(sorted(solution.robot_alignments.keys()))
    rejection_text = ",".join(
        f"{name}:{count}" for name, count in sorted(solution.rejection_counts.items())
    ) or "none"
    return (
        f"anchor={solution.anchor_robot} robots=[{aligned}] "
        f"received={solution.received_constraint_count} unique={solution.constraint_count} "
        f"used={solution.used_constraint_count} rejected={solution.rejected_constraint_count} "
        f"optimized_keyframes={len(solution.optimized_keyframes)} rejections=[{rejection_text}]"
    )
