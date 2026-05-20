from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Sim3Estimate:
    data: list[float]
    rotation: np.ndarray
    translation: np.ndarray
    scale: float
    inlier_count: int
    rmse: float
    residuals: np.ndarray
    inlier_mask: np.ndarray


class Sim3EstimationError(ValueError):
    pass


def estimate_umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or src.shape[0] < 3:
        raise Sim3EstimationError("Sim(3) estimation requires at least 3 paired 3D points")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    src_var = float(np.mean(np.sum(src_centered * src_centered, axis=1)))
    if src_var <= 1e-12:
        raise Sim3EstimationError("source points are degenerate")

    covariance = (dst_centered.T @ src_centered) / src.shape[0]
    u, singular_values, vt = np.linalg.svd(covariance)
    sign = np.ones(3, dtype=np.float64)
    if np.linalg.det(u @ vt) < 0.0:
        sign[-1] = -1.0
    rotation = u @ np.diag(sign) @ vt
    scale = float(np.sum(singular_values * sign) / src_var)
    if not np.isfinite(scale) or scale <= 0.0:
        raise Sim3EstimationError("estimated Sim(3) scale is invalid")

    translation = dst_mean - scale * (rotation @ src_mean)
    return scale, rotation, translation


def apply_sim3(src: np.ndarray, scale: float, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return scale * (src @ rotation.T) + translation.reshape(1, 3)


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    r = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    else:
        axis = int(np.argmax(np.diag(r)))
        if axis == 0:
            s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            qw = (r[2, 1] - r[1, 2]) / s
            qx = 0.25 * s
            qy = (r[0, 1] + r[1, 0]) / s
            qz = (r[0, 2] + r[2, 0]) / s
        elif axis == 1:
            s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            qw = (r[0, 2] - r[2, 0]) / s
            qx = (r[0, 1] + r[1, 0]) / s
            qy = 0.25 * s
            qz = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            qw = (r[1, 0] - r[0, 1]) / s
            qx = (r[0, 2] + r[2, 0]) / s
            qy = (r[1, 2] + r[2, 1]) / s
            qz = 0.25 * s

    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise Sim3EstimationError("invalid quaternion")
    return [float(x) for x in (quat / norm).tolist()]


def ransac_sim3(
    src: np.ndarray,
    dst: np.ndarray,
    iterations: int = 128,
    inlier_threshold: float = 0.20,
    min_inliers: int = 12,
    seed: int = 7,
) -> Sim3Estimate:
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    if src.shape[0] < max(3, int(min_inliers)):
        raise Sim3EstimationError(f"not enough 3D correspondences: {src.shape[0]}")

    rng = np.random.default_rng(seed)
    best_mask = None
    best_rmse = float("inf")
    sample_size = 3

    for _ in range(int(iterations)):
        sample = rng.choice(src.shape[0], size=sample_size, replace=False)
        try:
            scale, rotation, translation = estimate_umeyama(src[sample], dst[sample])
        except Sim3EstimationError:
            continue
        residuals = np.linalg.norm(apply_sim3(src, scale, rotation, translation) - dst, axis=1)
        mask = residuals <= float(inlier_threshold)
        inlier_count = int(mask.sum())
        if inlier_count < int(min_inliers):
            continue
        rmse = float(np.sqrt(np.mean(residuals[mask] ** 2)))
        if best_mask is None or inlier_count > int(best_mask.sum()) or (inlier_count == int(best_mask.sum()) and rmse < best_rmse):
            best_mask = mask
            best_rmse = rmse

    if best_mask is None:
        raise Sim3EstimationError("RANSAC found no valid Sim(3) consensus")

    scale, rotation, translation = estimate_umeyama(src[best_mask], dst[best_mask])
    residuals = np.linalg.norm(apply_sim3(src, scale, rotation, translation) - dst, axis=1)
    refined_mask = residuals <= float(inlier_threshold)
    inlier_count = int(refined_mask.sum())
    if inlier_count < int(min_inliers):
        raise Sim3EstimationError(f"too few Sim(3) inliers after refinement: {inlier_count}")
    rmse = float(np.sqrt(np.mean(residuals[refined_mask] ** 2)))
    quat = rotation_to_quaternion_xyzw(rotation)
    data = [float(translation[0]), float(translation[1]), float(translation[2]), *quat, float(scale)]
    return Sim3Estimate(
        data=data,
        rotation=rotation,
        translation=translation,
        scale=float(scale),
        inlier_count=inlier_count,
        rmse=rmse,
        residuals=residuals,
        inlier_mask=refined_mask,
    )
