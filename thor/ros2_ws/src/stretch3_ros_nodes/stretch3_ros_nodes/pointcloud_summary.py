from typing import Any, Dict

import numpy as np
from sensor_msgs.msg import PointCloud2


class PointCloudSummaryError(ValueError):
    pass


def summarize_cloud(msg: PointCloud2, max_points: int = 20000) -> Dict[str, Any]:
    field_offsets = {field.name: field.offset for field in msg.fields}
    for required in ("x", "y", "z"):
        if required not in field_offsets:
            raise PointCloudSummaryError(f"missing field {required}")

    point_count = int(msg.width) * int(msg.height)
    if point_count <= 0:
        raise PointCloudSummaryError("empty point cloud")

    dtype_fields = {
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
        "itemsize": int(msg.point_step),
    }
    if "confidence" in field_offsets:
        dtype_fields["names"].append("confidence")
        dtype_fields["formats"].append("<f4")
        dtype_fields["offsets"].append(field_offsets["confidence"])

    data = np.frombuffer(bytes(msg.data), dtype=np.dtype(dtype_fields), count=point_count)
    if max_points > 0 and point_count > max_points:
        step = max(1, point_count // max_points)
        data = data[::step][:max_points]

    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32, copy=False)
    finite_mask = np.isfinite(xyz).all(axis=1)
    finite = xyz[finite_mask]
    if finite.shape[0] == 0:
        raise PointCloudSummaryError("cloud has no finite xyz points")

    centroid = finite.mean(axis=0)
    radii = np.linalg.norm(finite - centroid, axis=1)
    result: Dict[str, Any] = {
        "frame_id": msg.header.frame_id,
        "point_count": point_count,
        "sampled_count": int(data.shape[0]),
        "finite_count": int(finite.shape[0]),
        "centroid": [float(x) for x in centroid.tolist()],
        "radius_mean": float(radii.mean()),
        "radius_p90": float(np.percentile(radii, 90)),
        "fields": [field.name for field in msg.fields],
    }

    if "confidence" in data.dtype.names:
        confidence = data["confidence"][finite_mask]
        confidence = confidence[np.isfinite(confidence)]
        if confidence.shape[0] > 0:
            result["confidence_mean"] = float(confidence.mean())
            result["confidence_p50"] = float(np.percentile(confidence, 50))

    return result
