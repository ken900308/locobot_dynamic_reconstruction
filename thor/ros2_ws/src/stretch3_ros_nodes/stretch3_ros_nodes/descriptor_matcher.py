import math
from typing import Iterable, List

from stretch3_ros_nodes.sim3_keyframe_types import LoopCandidate, Sim3Keyframe


def cosine_similarity(a: List[float], b: List[float]) -> float:
    if not a or not b or len(a) != len(b):
        return float("-inf")

    dot = 0.0
    norm_a = 0.0
    norm_b = 0.0
    for ai, bi in zip(a, b):
        dot += ai * bi
        norm_a += ai * ai
        norm_b += bi * bi

    denom = math.sqrt(norm_a) * math.sqrt(norm_b)
    if denom <= 0.0:
        return float("-inf")
    return dot / denom


class DescriptorMatcher:
    def __init__(self, min_similarity: float, top_k: int):
        self.min_similarity = min_similarity
        self.top_k = max(1, top_k)

    def find_candidates(
        self,
        query: Sim3Keyframe,
        references: Iterable[Sim3Keyframe],
    ) -> List[LoopCandidate]:
        candidates: List[LoopCandidate] = []
        for reference in references:
            score = cosine_similarity(query.descriptor, reference.descriptor)
            if score >= self.min_similarity:
                candidates.append(
                    LoopCandidate(
                        from_key=query.key,
                        to_key=reference.key,
                        similarity=score,
                    )
                )

        candidates.sort(key=lambda item: item.similarity, reverse=True)
        return candidates[: self.top_k]
