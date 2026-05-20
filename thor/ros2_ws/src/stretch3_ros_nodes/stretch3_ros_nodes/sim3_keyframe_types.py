import json
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


KeyframeKey = Tuple[str, int]


def key_to_uid(key: KeyframeKey) -> str:
    return f"{key[0]}_kf_{key[1]:06d}"


@dataclass(frozen=True)
class Sim3Keyframe:
    robot_id: str
    kf_id: int
    keyframe_uid: str
    sim3_data: List[float]
    descriptor: List[float]
    stamp: int
    confidence_mean: float | None
    num_points: int

    @property
    def key(self) -> KeyframeKey:
        return (self.robot_id, self.kf_id)


@dataclass(frozen=True)
class LoopCandidate:
    from_key: KeyframeKey
    to_key: KeyframeKey
    similarity: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": "inter_robot_loop_candidate_v1",
            "from_robot": self.from_key[0],
            "from_kf_id": self.from_key[1],
            "from_uid": key_to_uid(self.from_key),
            "to_robot": self.to_key[0],
            "to_kf_id": self.to_key[1],
            "to_uid": key_to_uid(self.to_key),
            "similarity": self.similarity,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))

    def to_summary(self, verification_ready: bool) -> str:
        ready = "ready" if verification_ready else "waiting"
        return (
            f"{key_to_uid(self.from_key)} -> {key_to_uid(self.to_key)} "
            f"similarity={self.similarity:.4f} verification={ready}"
        )


def parse_keyframe_metadata(text: str) -> Sim3Keyframe:
    data: Dict[str, Any] = json.loads(text)
    if data.get("schema") != "mast3r_keyframe_metadata_v1":
        raise ValueError(f"unsupported keyframe metadata schema: {data.get('schema')}")

    robot_id = str(data["robot_id"])
    kf_id = int(data["kf_id"])
    descriptor = [float(x) for x in data.get("descriptor", [])]
    sim3_data = [float(x) for x in data.get("sim3_data", [])]
    if len(sim3_data) != 8:
        raise ValueError(f"expected 8-value Sim(3), got {len(sim3_data)}")

    confidence = data.get("confidence_mean")
    return Sim3Keyframe(
        robot_id=robot_id,
        kf_id=kf_id,
        keyframe_uid=str(data.get("keyframe_uid", f"{robot_id}_kf_{kf_id:06d}")),
        sim3_data=sim3_data,
        descriptor=descriptor,
        stamp=int(data.get("stamp", 0)),
        confidence_mean=float(confidence) if confidence is not None else None,
        num_points=int(data.get("num_points", 0)),
    )
