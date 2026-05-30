import json
from typing import Any, Dict

from stretch3_ros_nodes.verification_job_types import GeometricVerificationJob


def make_pose_constraint_json(
    job: GeometricVerificationJob,
    relative_sim3_data: list[float],
    match_count: int,
    correspondence_count: int,
    inlier_count: int,
    rmse: float,
    confidence: float,
    verifier_backend: str = "",
) -> str:
    payload: Dict[str, Any] = {
        "schema": "inter_robot_sim3_constraint_v1",
        "candidate": job.candidate,
        "from_robot": job.from_key[0],
        "from_kf_id": job.from_key[1],
        "from_uid": job.from_uid,
        "to_robot": job.to_key[0],
        "to_kf_id": job.to_key[1],
        "to_uid": job.to_uid,
        "relative_sim3_data": [float(x) for x in relative_sim3_data],
        "relative_sim3_layout": "tx_ty_tz_qx_qy_qz_qw_scale",
        "transform_direction": "to_keyframe_local ~= relative_sim3_from_to * from_keyframe_local",
        "match_count": int(match_count),
        "correspondence_count": int(correspondence_count),
        "inlier_count": int(inlier_count),
        "inlier_ratio": float(int(inlier_count) / max(1, int(correspondence_count))),
        "rmse_m": float(rmse),
        "confidence": float(confidence),
        "verifier_backend": str(verifier_backend),
    }
    return json.dumps(payload, separators=(",", ":"))
