import json
import os
from typing import Any, Dict, List, Optional

import numpy as np
import torch

try:
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, PointCloud2, PointField
    from std_msgs.msg import Header, String

    ROS_METADATA_AVAILABLE = True
except Exception:
    HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    Image = PointCloud2 = PointField = Header = String = None
    ROS_METADATA_AVAILABLE = False


def _tensor_to_list(value: torch.Tensor) -> List[float]:
    return [float(x) for x in value.detach().cpu().reshape(-1).tolist()]


def _mean_pooled_descriptor(feat: Optional[torch.Tensor]) -> List[float]:
    if feat is None:
        return []

    with torch.no_grad():
        pooled = feat.detach().float().cpu().reshape(-1, feat.shape[-1]).mean(dim=0)
        norm = torch.linalg.norm(pooled)
        if float(norm) > 0.0:
            pooled = pooled / norm
        return _tensor_to_list(pooled)


def _uimg_to_rgb8(uimg: Any) -> np.ndarray | None:
    if uimg is None:
        return None

    if isinstance(uimg, torch.Tensor):
        arr = uimg.detach().cpu().numpy()
    else:
        arr = np.asarray(uimg)

    if arr.ndim == 4:
        arr = arr[0]
    if arr.ndim == 3 and arr.shape[0] in (1, 3, 4) and arr.shape[-1] not in (1, 3, 4):
        arr = np.moveaxis(arr, 0, -1)
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    if arr.ndim != 3 or arr.shape[2] < 3:
        return None

    arr = arr[:, :, :3]
    if arr.dtype != np.uint8:
        arr = np.nan_to_num(arr, nan=0.0, posinf=1.0, neginf=0.0)
        if float(np.nanmax(arr)) <= 1.5:
            arr = arr * 255.0
        arr = np.clip(arr, 0.0, 255.0).astype(np.uint8)
    return np.ascontiguousarray(arr)


def keyframe_to_metadata(robot_id: str, keyframe: Any) -> Dict[str, Any]:
    sim3_data = _tensor_to_list(keyframe.T_WC.data)
    descriptor = _mean_pooled_descriptor(getattr(keyframe, "feat", None))

    confidence_mean = None
    if getattr(keyframe, "C", None) is not None:
        confidence = _keyframe_average_confidence(keyframe)
        confidence_mean = float(confidence.detach().float().mean().cpu())

    image = _uimg_to_rgb8(getattr(keyframe, "uimg", None))
    image_shape = None
    if image is not None:
        image_shape = [int(image.shape[0]), int(image.shape[1]), int(image.shape[2])]

    return {
        "schema": "mast3r_keyframe_metadata_v1",
        "robot_id": robot_id,
        "kf_id": int(keyframe.frame_id),
        "keyframe_uid": f"{robot_id}_kf_{int(keyframe.frame_id):06d}",
        "sim3_data": sim3_data,
        "sim3_layout": "lietorch_sim3_data_tx_ty_tz_qx_qy_qz_qw_s",
        "descriptor": descriptor,
        "descriptor_type": "mast3r_feat_mean_l2_v1",
        "confidence_mean": confidence_mean,
        "confidence_type": "mast3r_average_confidence_C_over_N_v1",
        "pointmap_updates": int(getattr(keyframe, "N", 1) or 1),
        "num_points": int(keyframe.X_canon.shape[0]) if getattr(keyframe, "X_canon", None) is not None else 0,
        "image_shape": image_shape,
    }


def _keyframe_average_confidence(keyframe: Any) -> torch.Tensor:
    if hasattr(keyframe, "get_average_conf"):
        confidence = keyframe.get_average_conf()
        if confidence is not None:
            return confidence

    confidence = keyframe.C
    updates = max(1, int(getattr(keyframe, "N", 1) or 1))
    return confidence / updates


class KeyframeMetadataExporter:
    def __init__(self, node: Any, robot_id: str):
        self.node = node
        self.robot_id = robot_id
        self.enabled = ROS_METADATA_AVAILABLE and getattr(node, "ros_enabled", False)
        self.publisher = None
        self.cloud_publisher = None
        self.image_publisher = None
        self.topic = os.environ.get(
            "MAST3R_KEYFRAME_METADATA_TOPIC",
            f"/{robot_id}/mast3r/keyframe_metadata",
        )
        self.cloud_topic = os.environ.get(
            "MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC",
            f"/{robot_id}/mast3r/keyframe_cloud_local",
        )
        self.image_topic = os.environ.get(
            "MAST3R_KEYFRAME_IMAGE_TOPIC",
            f"/{robot_id}/mast3r/keyframe_image",
        )
        self.cloud_max_points = int(os.environ.get("MAST3R_KEYFRAME_LOCAL_CLOUD_MAX_POINTS", "0"))
        self.cloud_min_confidence = float(os.environ.get("MAST3R_KEYFRAME_LOCAL_CLOUD_MIN_CONFIDENCE", "0.0"))

        if not self.enabled:
            return

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.publisher = node.create_publisher(String, self.topic, qos)
        self.cloud_publisher = node.create_publisher(PointCloud2, self.cloud_topic, qos)
        self.image_publisher = node.create_publisher(Image, self.image_topic, qos)
        node.get_logger().info(f"Keyframe metadata publisher: {self.topic}")
        node.get_logger().info(f"Keyframe local cloud publisher: {self.cloud_topic}")
        node.get_logger().info(f"Keyframe image publisher: {self.image_topic}")

    def publish(self, keyframe: Any) -> None:
        if self.publisher is None:
            return

        payload = keyframe_to_metadata(self.robot_id, keyframe)
        payload["stamp"] = self.node.get_clock().now().nanoseconds

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(msg)
        self._publish_keyframe_image(keyframe, payload["keyframe_uid"])
        self._publish_local_cloud(keyframe, payload["keyframe_uid"])

    def _publish_keyframe_image(self, keyframe: Any, keyframe_uid: str) -> None:
        if self.image_publisher is None:
            return
        image = _uimg_to_rgb8(getattr(keyframe, "uimg", None))
        if image is None:
            return

        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = keyframe_uid
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(image.shape[1] * 3)
        msg.data = image.tobytes()
        self.image_publisher.publish(msg)

    def _publish_local_cloud(self, keyframe: Any, keyframe_uid: str) -> None:
        if self.cloud_publisher is None:
            return
        if getattr(keyframe, "X_canon", None) is None or getattr(keyframe, "C", None) is None:
            return

        points = keyframe.X_canon.detach().cpu()
        confidence = _keyframe_average_confidence(keyframe).detach().cpu().reshape(-1)
        valid = confidence >= self.cloud_min_confidence
        if int(valid.sum()) == 0:
            return

        indices = torch.nonzero(valid, as_tuple=False).reshape(-1)
        if self.cloud_max_points > 0 and indices.numel() > self.cloud_max_points:
            step = max(1, indices.numel() // self.cloud_max_points)
            indices = indices[::step][: self.cloud_max_points]

        points_np = points[indices].numpy().astype(np.float32)
        confidence_np = confidence[indices].numpy().astype(np.float32)
        colors_np = self._colors_for_indices(keyframe, indices)
        uv_np = self._uv_for_indices(keyframe, indices)

        point_dtype = np.dtype(
            [
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("rgb", "<u4"),
                ("confidence", "<f4"),
                ("u", "<u4"),
                ("v", "<u4"),
            ]
        )
        point_data = np.zeros(len(points_np), dtype=point_dtype)
        point_data["x"] = points_np[:, 0]
        point_data["y"] = points_np[:, 1]
        point_data["z"] = points_np[:, 2]
        r = colors_np[:, 0].astype(np.uint32)
        g = colors_np[:, 1].astype(np.uint32)
        b = colors_np[:, 2].astype(np.uint32)
        point_data["rgb"] = (r << 16) | (g << 8) | b
        point_data["confidence"] = confidence_np
        point_data["u"] = uv_np[:, 0]
        point_data["v"] = uv_np[:, 1]

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = keyframe_uid
        msg.height = 1
        msg.width = len(points_np)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
            PointField(name="confidence", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="u", offset=20, datatype=PointField.UINT32, count=1),
            PointField(name="v", offset=24, datatype=PointField.UINT32, count=1),
        ]
        msg.point_step = 28
        msg.row_step = msg.point_step * msg.width
        msg.data = point_data.tobytes()
        self.cloud_publisher.publish(msg)

    def _colors_for_indices(self, keyframe: Any, indices: torch.Tensor) -> np.ndarray:
        image = _uimg_to_rgb8(getattr(keyframe, "uimg", None))
        if image is None:
            return np.ones((indices.numel(), 3), dtype=np.uint8) * 255

        flat_colors = image.reshape(-1, 3)
        index_np = indices.numpy()
        if int(index_np.max(initial=0)) >= flat_colors.shape[0]:
            return np.ones((indices.numel(), 3), dtype=np.uint8) * 255
        return flat_colors[index_np]

    def _uv_for_indices(self, keyframe: Any, indices: torch.Tensor) -> np.ndarray:
        image = _uimg_to_rgb8(getattr(keyframe, "uimg", None))
        uv = np.zeros((indices.numel(), 2), dtype=np.uint32)
        if image is None or image.shape[1] <= 0:
            return uv

        width = int(image.shape[1])
        index_np = indices.numpy().astype(np.uint64)
        uv[:, 0] = (index_np % width).astype(np.uint32)
        uv[:, 1] = (index_np // width).astype(np.uint32)
        return uv
