from dataclasses import dataclass

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from stretch3_ros_nodes.pointcloud_voxel_filter import cap_points, voxel_downsample_points
from stretch3_ros_nodes.sim3_math import Sim3


class PointCloudTransformError(ValueError):
    pass


@dataclass(frozen=True)
class TransformedCloud:
    keyframe_uid: str
    point_count: int
    msg: PointCloud2


def transform_keyframe_cloud(
    msg: PointCloud2,
    transform: Sim3,
    output_frame: str,
    max_points: int = 60000,
    min_confidence: float = 0.0,
    voxel_leaf_size: float = 0.0,
) -> TransformedCloud:
    offsets = {field.name: field.offset for field in msg.fields}
    for required in ("x", "y", "z"):
        if required not in offsets:
            raise PointCloudTransformError(f"cloud missing field {required}")

    point_count = int(msg.width) * int(msg.height)
    if point_count <= 0:
        raise PointCloudTransformError("empty cloud")

    names = ["x", "y", "z"]
    formats = ["<f4", "<f4", "<f4"]
    field_offsets = [offsets["x"], offsets["y"], offsets["z"]]
    if "rgb" in offsets:
        names.append("rgb")
        formats.append("<u4")
        field_offsets.append(offsets["rgb"])
    if "confidence" in offsets:
        names.append("confidence")
        formats.append("<f4")
        field_offsets.append(offsets["confidence"])

    dtype = np.dtype({"names": names, "formats": formats, "offsets": field_offsets, "itemsize": int(msg.point_step)})
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=point_count)
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64, copy=False)
    valid = np.isfinite(xyz).all(axis=1)
    if "confidence" in data.dtype.names:
        valid = valid & np.isfinite(data["confidence"]) & (data["confidence"] >= float(min_confidence))

    indices = np.flatnonzero(valid)
    if indices.shape[0] == 0:
        raise PointCloudTransformError("cloud has no valid points")

    selected = xyz[indices]
    transformed = transform.scale * (selected @ transform.rotation.T) + transform.translation.reshape(1, 3)

    point_data = make_point_array(
        transformed.astype(np.float32),
        rgb_values=data["rgb"][indices] if "rgb" in data.dtype.names else None,
        confidence_values=data["confidence"][indices] if "confidence" in data.dtype.names else None,
    )
    point_data = voxel_downsample_points(point_data, voxel_leaf_size)
    point_data = cap_points(point_data, max_points)
    return make_cloud_from_point_array(point_data, msg.header.stamp, output_frame, msg.header.frame_id)


def point_dtype() -> np.dtype:
    return np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("rgb", "<u4"),
            ("confidence", "<f4"),
        ]
    )


def make_point_array(
    xyz: np.ndarray,
    rgb_values: np.ndarray | None,
    confidence_values: np.ndarray | None,
) -> np.ndarray:
    point_data = np.zeros(int(xyz.shape[0]), dtype=point_dtype())
    point_data["x"] = xyz[:, 0]
    point_data["y"] = xyz[:, 1]
    point_data["z"] = xyz[:, 2]
    if rgb_values is None:
        point_data["rgb"] = np.uint32(0xFFFFFF)
    else:
        point_data["rgb"] = rgb_values.astype(np.uint32, copy=False)
    if confidence_values is None:
        point_data["confidence"] = np.float32(1.0)
    else:
        point_data["confidence"] = confidence_values.astype(np.float32, copy=False)
    return point_data


def make_cloud_from_point_array(point_data: np.ndarray, stamp, frame_id: str, keyframe_uid: str) -> TransformedCloud:
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="confidence", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    out = PointCloud2()
    out.header = Header()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.height = 1
    out.width = int(point_data.shape[0])
    out.fields = fields
    out.is_bigendian = False
    out.point_step = 20
    out.row_step = out.point_step * out.width
    out.is_dense = True
    out.data = point_data.tobytes()
    return TransformedCloud(keyframe_uid=keyframe_uid, point_count=int(point_data.shape[0]), msg=out)


def merge_transformed_clouds(
    clouds: list[TransformedCloud],
    output_frame: str,
    stamp,
    max_points: int = 300000,
    voxel_leaf_size: float = 0.0,
) -> PointCloud2 | None:
    if not clouds:
        return None
    arrays = []
    dtype = point_dtype()
    for cloud in clouds:
        count = int(cloud.msg.width) * int(cloud.msg.height)
        if count <= 0:
            continue
        arrays.append(np.frombuffer(bytes(cloud.msg.data), dtype=dtype, count=count))
    if not arrays:
        return None
    merged = np.concatenate(arrays)
    merged = voxel_downsample_points(merged, voxel_leaf_size)
    merged = cap_points(merged, max_points)
    return make_cloud_from_point_array(merged, stamp, output_frame, "optimized_map").msg
