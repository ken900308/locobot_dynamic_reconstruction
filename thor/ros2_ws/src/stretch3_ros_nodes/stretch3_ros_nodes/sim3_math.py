from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Sim3:
    translation: np.ndarray
    rotation: np.ndarray
    scale: float

    @staticmethod
    def identity() -> "Sim3":
        return Sim3(np.zeros(3, dtype=np.float64), np.eye(3, dtype=np.float64), 1.0)

    @staticmethod
    def from_list(data: list[float] | tuple[float, ...]) -> "Sim3":
        if len(data) != 8:
            raise ValueError(f"expected 8-value Sim(3), got {len(data)}")
        translation = np.asarray(data[:3], dtype=np.float64)
        rotation = quaternion_xyzw_to_rotation(data[3:7])
        scale = float(data[7])
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError(f"invalid Sim(3) scale: {scale}")
        return Sim3(translation, rotation, scale)

    def to_list(self) -> list[float]:
        quat = rotation_to_quaternion_xyzw(self.rotation)
        return [float(self.translation[0]), float(self.translation[1]), float(self.translation[2]), *quat, float(self.scale)]

    def inverse(self) -> "Sim3":
        inv_scale = 1.0 / self.scale
        inv_rotation = self.rotation.T
        inv_translation = -inv_scale * (inv_rotation @ self.translation)
        return Sim3(inv_translation, inv_rotation, inv_scale)

    def compose(self, other: "Sim3") -> "Sim3":
        scale = self.scale * other.scale
        rotation = self.rotation @ other.rotation
        translation = self.scale * (self.rotation @ other.translation) + self.translation
        return Sim3(translation, rotation, scale)


def quaternion_xyzw_to_rotation(quat: list[float] | tuple[float, ...] | np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError("invalid zero quaternion")
    x, y, z, w = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    r = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        axis = int(np.argmax(np.diag(r)))
        if axis == 0:
            s = math.sqrt(max(1e-12, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif axis == 1:
            s = math.sqrt(max(1e-12, 1.0 + r[1, 1] - r[0, 0] - r[2, 2])) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(max(1e-12, 1.0 + r[2, 2] - r[0, 0] - r[1, 1])) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    q /= max(1e-12, float(np.linalg.norm(q)))
    if q[3] < 0.0:
        q = -q
    return [float(x) for x in q.tolist()]


def average_sim3(transforms: list[Sim3], weights: list[float] | None = None) -> Sim3:
    if not transforms:
        raise ValueError("cannot average zero Sim(3) transforms")
    w = np.ones(len(transforms), dtype=np.float64) if weights is None else np.asarray(weights, dtype=np.float64)
    w = np.maximum(w, 1e-9)
    w /= float(w.sum())

    translation = np.zeros(3, dtype=np.float64)
    log_scale = 0.0
    quat_outer = np.zeros((4, 4), dtype=np.float64)
    ref_quat = np.asarray(rotation_to_quaternion_xyzw(transforms[0].rotation), dtype=np.float64)

    for weight, transform in zip(w, transforms):
        translation += weight * transform.translation
        log_scale += weight * math.log(transform.scale)
        quat = np.asarray(rotation_to_quaternion_xyzw(transform.rotation), dtype=np.float64)
        if float(np.dot(ref_quat, quat)) < 0.0:
            quat = -quat
        quat_outer += weight * np.outer(quat, quat)

    _, eigenvectors = np.linalg.eigh(quat_outer)
    quat = eigenvectors[:, -1]
    if quat[3] < 0.0:
        quat = -quat
    rotation = quaternion_xyzw_to_rotation(quat)
    return Sim3(translation, rotation, float(math.exp(log_scale)))
