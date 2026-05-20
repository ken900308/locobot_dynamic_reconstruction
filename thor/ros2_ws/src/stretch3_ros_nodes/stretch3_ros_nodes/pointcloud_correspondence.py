from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2


class PointCloudCorrespondenceError(ValueError):
    pass


@dataclass
class PointCloudPixelIndex:
    points_by_uv: dict[tuple[int, int], np.ndarray]
    point_count: int
    finite_count: int

    def nearest_xyz(self, u: float, v: float, max_px: int) -> np.ndarray | None:
        center_u = int(round(u))
        center_v = int(round(v))
        for radius in range(int(max_px) + 1):
            for dv in range(-radius, radius + 1):
                for du in range(-radius, radius + 1):
                    if max(abs(du), abs(dv)) != radius:
                        continue
                    point = self.points_by_uv.get((center_u + du, center_v + dv))
                    if point is not None:
                        return point
        return None


def build_pixel_index(msg: PointCloud2, min_confidence: float = 0.0) -> PointCloudPixelIndex:
    offsets = {field.name: field.offset for field in msg.fields}
    for required in ("x", "y", "z", "u", "v"):
        if required not in offsets:
            raise PointCloudCorrespondenceError(f"cloud is missing pixel field {required}")

    point_count = int(msg.width) * int(msg.height)
    if point_count <= 0:
        raise PointCloudCorrespondenceError("empty cloud")

    names = ["x", "y", "z", "u", "v"]
    formats = ["<f4", "<f4", "<f4", "<u4", "<u4"]
    field_offsets = [offsets[name] for name in names]
    if "confidence" in offsets:
        names.append("confidence")
        formats.append("<f4")
        field_offsets.append(offsets["confidence"])

    dtype = np.dtype({"names": names, "formats": formats, "offsets": field_offsets, "itemsize": int(msg.point_step)})
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=point_count)
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float32, copy=False)
    finite = np.isfinite(xyz).all(axis=1)
    if "confidence" in data.dtype.names:
        finite = finite & np.isfinite(data["confidence"]) & (data["confidence"] >= float(min_confidence))

    points_by_uv: dict[tuple[int, int], np.ndarray] = {}
    finite_indices = np.flatnonzero(finite)
    for idx in finite_indices:
        key = (int(data["u"][idx]), int(data["v"][idx]))
        points_by_uv[key] = xyz[idx].astype(np.float64, copy=True)

    if not points_by_uv:
        raise PointCloudCorrespondenceError("cloud has no finite pixel-indexed xyz points")

    return PointCloudPixelIndex(points_by_uv=points_by_uv, point_count=point_count, finite_count=len(points_by_uv))
