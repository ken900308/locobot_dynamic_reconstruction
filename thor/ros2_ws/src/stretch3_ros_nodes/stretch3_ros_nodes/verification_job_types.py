import json
from dataclasses import dataclass
from typing import Any, Dict, Tuple

from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, key_to_uid


@dataclass(frozen=True)
class GeometricVerificationJob:
    candidate: Dict[str, Any]
    from_key: KeyframeKey
    to_key: KeyframeKey
    from_uid: str
    to_uid: str
    from_sim3_data: list[float]
    to_sim3_data: list[float]


def parse_verification_job(text: str) -> GeometricVerificationJob:
    data: Dict[str, Any] = json.loads(text)
    if data.get("schema") != "geometric_verification_job_v1":
        raise ValueError(f"unsupported verification job schema: {data.get('schema')}")

    candidate = data["candidate"]
    from_key = (str(candidate["from_robot"]), int(candidate["from_kf_id"]))
    to_key = (str(candidate["to_robot"]), int(candidate["to_kf_id"]))
    from_sim3 = [float(x) for x in data.get("from_sim3_data", [])]
    to_sim3 = [float(x) for x in data.get("to_sim3_data", [])]
    if len(from_sim3) != 8 or len(to_sim3) != 8:
        raise ValueError("verification job requires 8-value Sim(3) poses")

    return GeometricVerificationJob(
        candidate=candidate,
        from_key=from_key,
        to_key=to_key,
        from_uid=str(data.get("from_uid", key_to_uid(from_key))),
        to_uid=str(data.get("to_uid", key_to_uid(to_key))),
        from_sim3_data=from_sim3,
        to_sim3_data=to_sim3,
    )


def make_result_json(
    job: GeometricVerificationJob,
    status: str,
    reason: str,
    constraint_ready: bool,
    from_summary: Dict[str, Any] | None = None,
    to_summary: Dict[str, Any] | None = None,
) -> str:
    payload = {
        "schema": "geometric_verification_result_v1",
        "status": status,
        "reason": reason,
        "constraint_ready": constraint_ready,
        "candidate": job.candidate,
        "from_uid": job.from_uid,
        "to_uid": job.to_uid,
        "from_cloud_summary": from_summary,
        "to_cloud_summary": to_summary,
    }
    return json.dumps(payload, separators=(",", ":"))
