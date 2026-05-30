from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from stretch3_ros_nodes.pgo_types import PoseConstraint
from stretch3_ros_nodes.sim3_keyframe_types import KeyframeKey, Sim3Keyframe
from stretch3_ros_nodes.sim3_math import Sim3, average_sim3


@dataclass(frozen=True)
class PgoSolveConfig:
    min_confidence: float
    max_rmse_m: float
    min_inliers: int
    min_inlier_ratio: float
    max_constraints_per_keyframe: int
    min_used_constraints_per_pair: int
    max_alignment_translation_residual_m: float
    max_alignment_rotation_residual_deg: float
    max_alignment_log_scale_residual: float
    min_pair_observations_for_robust: int


@dataclass(frozen=True)
class AlignmentObservation:
    transform: Sim3
    weight: float
    constraint: PoseConstraint


@dataclass(frozen=True)
class PgoSolution:
    anchor_robot: str
    robot_alignments: Dict[str, Sim3]
    optimized_keyframes: Dict[KeyframeKey, Sim3]
    constraint_count: int
    used_constraint_count: int
    received_constraint_count: int
    rejected_constraint_count: int
    rejection_counts: Dict[str, int] = field(default_factory=dict)
    pair_observation_counts: Dict[str, int] = field(default_factory=dict)
    pair_used_counts: Dict[str, int] = field(default_factory=dict)


class Sim3AlignmentGraph:
    def __init__(self, anchor_robot: str):
        self.anchor_robot = anchor_robot
        self.keyframes: Dict[KeyframeKey, Sim3Keyframe] = {}
        self.constraints: Dict[tuple[KeyframeKey, KeyframeKey], PoseConstraint] = {}
        self.received_constraint_count = 0

    def upsert_keyframe(self, keyframe: Sim3Keyframe) -> bool:
        is_new = keyframe.key not in self.keyframes
        self.keyframes[keyframe.key] = keyframe
        return is_new

    def upsert_constraint(self, constraint: PoseConstraint) -> bool:
        self.received_constraint_count += 1
        is_new = constraint.key not in self.constraints
        self.constraints[constraint.key] = constraint
        return is_new

    def solve(
        self,
        min_confidence: float,
        max_rmse_m: float,
        min_inliers: int,
        min_inlier_ratio: float = 0.55,
        max_constraints_per_keyframe: int = 1,
        min_used_constraints_per_pair: int = 2,
        max_alignment_translation_residual_m: float = 1.0,
        max_alignment_rotation_residual_deg: float = 30.0,
        max_alignment_log_scale_residual: float = 0.25,
        min_pair_observations_for_robust: int = 3,
    ) -> PgoSolution:
        config = PgoSolveConfig(
            min_confidence=min_confidence,
            max_rmse_m=max_rmse_m,
            min_inliers=min_inliers,
            min_inlier_ratio=min_inlier_ratio,
            max_constraints_per_keyframe=max(0, int(max_constraints_per_keyframe)),
            min_used_constraints_per_pair=max(1, int(min_used_constraints_per_pair)),
            max_alignment_translation_residual_m=max_alignment_translation_residual_m,
            max_alignment_rotation_residual_deg=max_alignment_rotation_residual_deg,
            max_alignment_log_scale_residual=max_alignment_log_scale_residual,
            min_pair_observations_for_robust=max(1, min_pair_observations_for_robust),
        )
        canonical_observations: Dict[tuple[str, str], list[AlignmentObservation]] = {}
        rejection_counts: Dict[str, int] = {}

        for constraint in self.constraints.values():
            observation = self._make_observation_or_reject(constraint, config, rejection_counts)
            if observation is None:
                continue
            pair = tuple(sorted((constraint.from_key[0], constraint.to_key[0])))
            if (constraint.from_key[0], constraint.to_key[0]) == pair:
                canonical_observation = observation
            else:
                canonical_observation = AlignmentObservation(
                    observation.transform.inverse(),
                    observation.weight,
                    constraint,
                )
            canonical_observations.setdefault(pair, []).append(canonical_observation)

        pair_edges: Dict[tuple[str, str], Sim3] = {}
        pair_observation_counts: Dict[str, int] = {}
        pair_used_counts: Dict[str, int] = {}
        used_constraints: set[tuple[KeyframeKey, KeyframeKey]] = set()

        for pair, observations in canonical_observations.items():
            selected = self._select_consensus_observations(
                observations,
                config,
                rejection_counts,
                count_rejections=True,
            )
            pair_observation_counts[self._pair_name(pair)] = len(observations)
            selected = self._select_best_keyframe_observations(selected, config, rejection_counts)
            pair_used_counts[self._pair_name(pair)] = len(selected)
            reverse_pair = (pair[1], pair[0])
            pair_observation_counts[self._pair_name(reverse_pair)] = len(observations)
            pair_used_counts[self._pair_name(reverse_pair)] = len(selected)
            if len(selected) < config.min_used_constraints_per_pair:
                for _item in selected:
                    self._count_rejection(rejection_counts, "too_few_pair_constraints")
                continue

            transforms = [item.transform for item in selected]
            weights = [item.weight for item in selected]
            forward = average_sim3(transforms, weights)
            pair_edges[pair] = forward
            pair_edges[reverse_pair] = forward.inverse()
            for item in selected:
                used_constraints.add(item.constraint.key)

        alignments = self._propagate_alignments(pair_edges)
        optimized: Dict[KeyframeKey, Sim3] = {}
        for key, keyframe in self.keyframes.items():
            alignment = alignments.get(keyframe.robot_id)
            if alignment is None:
                continue
            optimized[key] = alignment.compose(Sim3.from_list(keyframe.sim3_data))

        used_count = len(used_constraints)
        rejected_count = max(0, len(self.constraints) - used_count)
        return PgoSolution(
            anchor_robot=self.anchor_robot,
            robot_alignments=alignments,
            optimized_keyframes=optimized,
            constraint_count=len(self.constraints),
            used_constraint_count=used_count,
            received_constraint_count=self.received_constraint_count,
            rejected_constraint_count=rejected_count,
            rejection_counts=rejection_counts,
            pair_observation_counts=pair_observation_counts,
            pair_used_counts=pair_used_counts,
        )

    def _make_observation_or_reject(
        self,
        constraint: PoseConstraint,
        config: PgoSolveConfig,
        rejection_counts: Dict[str, int],
    ) -> AlignmentObservation | None:
        if constraint.confidence < config.min_confidence:
            self._count_rejection(rejection_counts, "low_confidence")
            return None
        if config.max_rmse_m > 0.0 and constraint.rmse_m > config.max_rmse_m:
            self._count_rejection(rejection_counts, "high_rmse")
            return None
        if constraint.inlier_count < config.min_inliers:
            self._count_rejection(rejection_counts, "too_few_inliers")
            return None
        if constraint.inlier_ratio < config.min_inlier_ratio:
            self._count_rejection(rejection_counts, "low_inlier_ratio")
            return None
        from_kf = self.keyframes.get(constraint.from_key)
        to_kf = self.keyframes.get(constraint.to_key)
        if from_kf is None or to_kf is None:
            self._count_rejection(rejection_counts, "missing_keyframe_metadata")
            return None
        if from_kf.robot_id == to_kf.robot_id:
            self._count_rejection(rejection_counts, "same_robot")
            return None

        from_pose = Sim3.from_list(from_kf.sim3_data)
        to_pose = Sim3.from_list(to_kf.sim3_data)
        relative = Sim3.from_list(constraint.relative_sim3_data)
        to_inv_from = to_pose.compose(relative).compose(from_pose.inverse())
        from_to_alignment = to_inv_from.inverse()
        return AlignmentObservation(from_to_alignment, self._constraint_weight(constraint), constraint)

    def _select_consensus_observations(
        self,
        observations: list[AlignmentObservation],
        config: PgoSolveConfig,
        rejection_counts: Dict[str, int],
        count_rejections: bool,
    ) -> list[AlignmentObservation]:
        if len(observations) < config.min_pair_observations_for_robust:
            return observations

        center = average_sim3(
            [item.transform for item in observations],
            [item.weight for item in observations],
        )
        selected = []
        for observation in observations:
            residual = sim3_residual(center, observation.transform)
            if residual["translation_m"] > config.max_alignment_translation_residual_m:
                if count_rejections:
                    self._count_rejection(rejection_counts, "alignment_translation_outlier")
                continue
            if residual["rotation_deg"] > config.max_alignment_rotation_residual_deg:
                if count_rejections:
                    self._count_rejection(rejection_counts, "alignment_rotation_outlier")
                continue
            if abs(residual["log_scale"]) > config.max_alignment_log_scale_residual:
                if count_rejections:
                    self._count_rejection(rejection_counts, "alignment_scale_outlier")
                continue
            selected.append(observation)
        return selected


    def _select_best_keyframe_observations(
        self,
        observations: list[AlignmentObservation],
        config: PgoSolveConfig,
        rejection_counts: Dict[str, int],
    ) -> list[AlignmentObservation]:
        limit = int(config.max_constraints_per_keyframe)
        if limit <= 0 or len(observations) <= 1:
            return observations

        selected = []
        key_use_counts: Dict[KeyframeKey, int] = {}
        for observation in sorted(observations, key=lambda item: item.weight, reverse=True):
            from_key = observation.constraint.from_key
            to_key = observation.constraint.to_key
            if key_use_counts.get(from_key, 0) >= limit or key_use_counts.get(to_key, 0) >= limit:
                self._count_rejection(rejection_counts, "keyframe_quality_superseded")
                continue
            selected.append(observation)
            key_use_counts[from_key] = key_use_counts.get(from_key, 0) + 1
            key_use_counts[to_key] = key_use_counts.get(to_key, 0) + 1
        return selected

    def _propagate_alignments(self, pair_edges: Dict[tuple[str, str], Sim3]) -> Dict[str, Sim3]:
        alignments = {self.anchor_robot: Sim3.identity()}
        pending = [self.anchor_robot]
        while pending:
            current = pending.pop(0)
            for (src, dst), transform in pair_edges.items():
                if src != current or dst in alignments:
                    continue
                alignments[dst] = alignments[src].compose(transform)
                pending.append(dst)
        return alignments

    def _constraint_weight(self, constraint: PoseConstraint) -> float:
        rmse_weight = 1.0 / max(1e-4, constraint.rmse_m * constraint.rmse_m)
        return max(1e-6, constraint.confidence) * max(1, constraint.inlier_count) * rmse_weight

    def _count_rejection(self, rejection_counts: Dict[str, int], reason: str) -> None:
        rejection_counts[reason] = rejection_counts.get(reason, 0) + 1

    def _pair_name(self, pair: tuple[str, str]) -> str:
        return f"{pair[0]}->{pair[1]}"


def sim3_residual(reference: Sim3, observed: Sim3) -> dict[str, float]:
    delta = reference.inverse().compose(observed)
    return {
        "translation_m": float(np.linalg.norm(delta.translation)),
        "rotation_deg": rotation_angle_deg(delta.rotation),
        "log_scale": float(np.log(max(1e-12, delta.scale))),
    }


def rotation_angle_deg(rotation: np.ndarray) -> float:
    trace = float(np.trace(rotation))
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return float(np.degrees(np.arccos(cos_angle)))
