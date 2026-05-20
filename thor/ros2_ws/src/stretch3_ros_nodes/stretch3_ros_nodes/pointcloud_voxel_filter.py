from __future__ import annotations

import numpy as np


def voxel_downsample_points(point_data: np.ndarray, leaf_size: float) -> np.ndarray:
    """Return one representative point per voxel, preferring higher confidence."""
    if leaf_size <= 0.0 or point_data.shape[0] == 0:
        return point_data

    xyz = np.stack([point_data["x"], point_data["y"], point_data["z"]], axis=1).astype(np.float64, copy=False)
    valid = np.isfinite(xyz).all(axis=1)
    if not np.any(valid):
        return point_data[:0]

    valid_indices = np.flatnonzero(valid)
    valid_xyz = xyz[valid_indices]
    voxels = np.floor(valid_xyz / float(leaf_size)).astype(np.int64)

    order_values = np.lexsort((voxels[:, 2], voxels[:, 1], voxels[:, 0]))
    sorted_indices = valid_indices[order_values]
    sorted_voxels = voxels[order_values]

    confidence = None
    if "confidence" in point_data.dtype.names:
        confidence = point_data["confidence"]

    selected = []
    start = 0
    while start < sorted_voxels.shape[0]:
        end = start + 1
        while end < sorted_voxels.shape[0] and np.array_equal(sorted_voxels[end], sorted_voxels[start]):
            end += 1
        candidates = sorted_indices[start:end]
        if confidence is None:
            selected.append(int(candidates[0]))
        else:
            best_local = int(np.argmax(confidence[candidates]))
            selected.append(int(candidates[best_local]))
        start = end

    return point_data[np.array(selected, dtype=np.int64)]


def cap_points(point_data: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or point_data.shape[0] <= max_points:
        return point_data
    step = max(1, point_data.shape[0] // int(max_points))
    return point_data[::step][: int(max_points)]
