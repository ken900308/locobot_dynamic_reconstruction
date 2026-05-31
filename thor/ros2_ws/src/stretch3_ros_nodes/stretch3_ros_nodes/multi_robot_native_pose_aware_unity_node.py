#!/usr/bin/env python3
from collections import deque
from functools import partial
import json
import re
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_native_cache import (
    NativeKeyframeManifest,
    manifest_from_metadata_json,
    validate_native_cache_manifest,
)
from stretch3_ros_nodes.native_frame_cache import load_frame_record, record_to_frame
from stretch3_ros_nodes.native_imports import configure_mast3r_imports
from stretch3_ros_nodes.native_messages import dumps
from stretch3_ros_nodes.pointcloud_voxel_filter import cap_points, voxel_downsample_points


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _sim3_to_list(value) -> list[float]:
    if hasattr(value, "detach"):
        value = value.detach().cpu().reshape(-1).tolist()
    elif hasattr(value, "reshape"):
        value = value.reshape(-1).tolist()
    return [float(x) for x in value]


def make_unity_xyzrgb_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    out = PointCloud2()
    out.header = Header()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.height = 1
    out.width = int(points.shape[0])
    out.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    out.is_bigendian = False
    out.point_step = 16
    out.row_step = out.point_step * out.width
    out.is_dense = True
    out.data = points.tobytes()
    return out


class MultiRobotNativePoseAwareUnityNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_pose_aware_unity_node")
        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("metadata_topic_template", "/{robot_id}/mast3r/keyframe_metadata")
        self.declare_parameter("native_cache_root", "/workspace/shared_native_keyframe_cache")
        self.declare_parameter("scan_cache_on_start", True)
        self.declare_parameter("scan_period_sec", 1.0)
        self.declare_parameter("optimized_pose_topic", "/multi_robot/native_optimized_keyframe_poses")
        self.declare_parameter("keyframe_cloud_topic", "/multi_robot/native_unity_keyframe_clouds")
        self.declare_parameter("pose_graph_topic", "/multi_robot/native_unity_pose_graph")
        self.declare_parameter("summary_topic", "/multi_robot/native_unity_summaries")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("min_confidence", 0.95)
        self.declare_parameter("max_points_per_keyframe", 0)
        self.declare_parameter("voxel_leaf_size", 0.0)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("cloud_publish_period_sec", 0.25)
        self.declare_parameter("max_clouds_per_tick", 1)

        configure_mast3r_imports(self.get_parameter("mast3r_slam_root").value)
        self.robot_ids = _parse_csv(self.get_parameter("robot_ids").value) or ["robot1", "robot2"]
        self.robot_order = {robot_id: i for i, robot_id in enumerate(self.robot_ids)}
        self.metadata_topic_template = self.get_parameter("metadata_topic_template").value
        self.native_cache_root = Path(str(self.get_parameter("native_cache_root").value))
        self.scan_cache_on_start = bool(self.get_parameter("scan_cache_on_start").value)
        self.scan_period_sec = float(self.get_parameter("scan_period_sec").value)
        self.device = self.get_parameter("device").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.max_points_per_keyframe = int(self.get_parameter("max_points_per_keyframe").value)
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.cloud_publish_period_sec = float(self.get_parameter("cloud_publish_period_sec").value)
        self.max_clouds_per_tick = max(1, int(self.get_parameter("max_clouds_per_tick").value))
        self.records = {}
        self.poses = {}
        self.sent_clouds = set()
        self.queued_clouds = set()
        self.pending_cloud_keys = deque()
        self.seen_cache_paths = set()
        self.pose_revision = 0
        self.cloud_count = 0
        self.dirty_poses = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.cloud_pub = self.create_publisher(PointCloud2, self.get_parameter("keyframe_cloud_topic").value, qos)
        self.pose_pub = self.create_publisher(String, self.get_parameter("pose_graph_topic").value, qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("summary_topic").value, qos)
        self._metadata_subscriptions = []
        for robot_id in self.robot_ids:
            topic = self.metadata_topic_template.format(robot_id=robot_id)
            self._metadata_subscriptions.append(self.create_subscription(String, topic, partial(self.on_metadata, robot_id), qos))
            self.get_logger().info(f"Listening for pose-aware keyframe metadata from {robot_id} on {topic}")
        self.sub = self.create_subscription(String, self.get_parameter("optimized_pose_topic").value, self.on_pose, qos)
        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self.publish_pose_graph) if period > 0 else None
        self.scan_timer = self.create_timer(self.scan_period_sec, self.scan_cache) if self.scan_cache_on_start and self.scan_period_sec > 0 else None
        self.cloud_timer = self.create_timer(self.cloud_publish_period_sec, self.publish_pending_clouds) if self.cloud_publish_period_sec > 0 else None
        self.get_logger().info(
            f"Native pose-aware Unity publisher listening on {self.get_parameter('optimized_pose_topic').value}; "
            f"metadata={self.metadata_topic_template}, cache_root={self.native_cache_root}, "
            f"clouds={self.get_parameter('keyframe_cloud_topic').value}, poses={self.get_parameter('pose_graph_topic').value}, "
            f"min_confidence={self.min_confidence}, max_points_per_keyframe={self.max_points_per_keyframe}, voxel={self.voxel_leaf_size}, "
            f"cloud_publish_period={self.cloud_publish_period_sec}, max_clouds_per_tick={self.max_clouds_per_tick}"
        )

    def _global_index(self, robot_id: str, kf_id: int) -> int:
        return self.robot_order.get(robot_id, len(self.robot_order)) * 1000000 + int(kf_id)

    def _manifest_from_cache_path(self, path: Path) -> NativeKeyframeManifest | None:
        match = re.search(r"(.+)_kf_(\d+)\.pt$", path.name)
        if match is None:
            return None
        robot_id = path.parent.name
        if robot_id not in self.robot_order:
            return None
        kf_id = int(match.group(2))
        keyframe_uid = path.stem
        return NativeKeyframeManifest(robot_id, kf_id, keyframe_uid, str(path), "mast3r_native_keyframe_cache_v1", {})

    def scan_cache(self):
        if not self.native_cache_root.exists():
            return
        discovered = 0
        for robot_id in self.robot_ids:
            robot_dir = self.native_cache_root / robot_id
            if not robot_dir.exists():
                continue
            for path in sorted(robot_dir.glob(f"{robot_id}_kf_*.pt")):
                cache_path = str(path)
                if cache_path in self.seen_cache_paths:
                    continue
                manifest = self._manifest_from_cache_path(path)
                if manifest is None:
                    continue
                if self._accept_manifest(manifest, source="cache_scan"):
                    discovered += 1
        if discovered:
            self.get_logger().info(f"Pose-aware cache scan discovered {discovered} keyframes; records={len(self.records)} sent_clouds={len(self.sent_clouds)}")

    def on_metadata(self, expected_robot_id: str, msg: String):
        try:
            manifest = manifest_from_metadata_json(msg.data)
            if manifest.robot_id != expected_robot_id:
                raise ValueError(f"robot_id mismatch: topic={expected_robot_id}, payload={manifest.robot_id}")
            self._accept_manifest(manifest, source="metadata")
        except Exception as exc:
            self.get_logger().warn(f"Rejected pose-aware keyframe metadata: {exc}")

    def _accept_manifest(self, manifest: NativeKeyframeManifest, source: str) -> bool:
        validate_native_cache_manifest(manifest)
        key = (manifest.robot_id, int(manifest.kf_id))
        self.seen_cache_paths.add(str(manifest.cache_path))
        if key not in self.records:
            record = load_frame_record(
                manifest,
                self._global_index(manifest.robot_id, manifest.kf_id),
                self.device,
                tensor_keys=("sim3_data", "uimg", "X_canon", "C"),
            )
            self.records[key] = record
            self.poses.setdefault(key, {
                "robot_id": record.robot_id,
                "kf_id": record.kf_id,
                "keyframe_uid": record.keyframe_uid,
                "global_index": record.global_index,
                "cache_path": record.cache_path,
                "optimized_sim3_data": _sim3_to_list(record.payload["sim3_data"]),
                "sim3_layout": "tx_ty_tz_qx_qy_qz_qw_scale",
                "pose_source": "native_cache_initial",
            })
        self.queue_keyframe_cloud(key)
        self.dirty_poses = True
        return True

    def on_pose(self, msg: String):
        try:
            data = json.loads(msg.data)
            key = (data["robot_id"], int(data["kf_id"]))
            self.poses[key] = data
            if key not in self.records:
                manifest = NativeKeyframeManifest(
                    data["robot_id"], int(data["kf_id"]), data["keyframe_uid"], data["cache_path"],
                    "mast3r_native_keyframe_cache_v1", {}
                )
                self.records[key] = load_frame_record(
                    manifest,
                    int(data.get("global_index", self._global_index(data["robot_id"], int(data["kf_id"])))),
                    self.device,
                    tensor_keys=("sim3_data", "uimg", "X_canon", "C"),
                )
            self.queue_keyframe_cloud(key)
            self.dirty_poses = True
        except Exception as exc:
            self.get_logger().warn(f"Rejected pose-aware Unity pose: {exc}")

    def queue_keyframe_cloud(self, key):
        if key in self.sent_clouds or key in self.queued_clouds:
            return
        self.pending_cloud_keys.append(key)
        self.queued_clouds.add(key)

    def publish_pending_clouds(self):
        published = 0
        while self.pending_cloud_keys and published < self.max_clouds_per_tick:
            key = self.pending_cloud_keys.popleft()
            self.queued_clouds.discard(key)
            if key in self.sent_clouds:
                continue
            try:
                if self.publish_keyframe_cloud(key):
                    published += 1
            except Exception as exc:
                self.get_logger().warn(f"Failed to publish queued pose-aware cloud {key}: {exc}")

    def publish_keyframe_cloud(self, key):
        record = self.records[key]
        frame = record_to_frame(record)
        x_local = frame.X_canon.detach().cpu().numpy().reshape(-1, 3)
        conf = frame.get_average_conf().detach().cpu().numpy().reshape(-1)
        valid = np.isfinite(x_local).all(axis=1) & np.isfinite(conf) & (conf >= self.min_confidence)
        if not np.any(valid):
            self.get_logger().warn(f"No valid native local points for {record.keyframe_uid}")
            return False
        xyz = x_local[valid].astype(np.float32)
        colors = self._colors(frame, valid)
        rgb = (colors[:, 0].astype(np.uint32) << 16) | (colors[:, 1].astype(np.uint32) << 8) | colors[:, 2].astype(np.uint32)
        points = np.zeros(int(xyz.shape[0]), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        points["x"] = xyz[:, 0]
        points["y"] = xyz[:, 1]
        points["z"] = xyz[:, 2]
        points["rgb"] = rgb
        points = voxel_downsample_points(points, self.voxel_leaf_size)
        points = cap_points(points, self.max_points_per_keyframe)
        frame_id = f"kf_{int(record.global_index)}"
        cloud = make_unity_xyzrgb_cloud(points, self.get_clock().now().to_msg(), frame_id)
        self.cloud_pub.publish(cloud)
        record.payload.pop("X_canon", None)
        record.payload.pop("C", None)
        record.payload.pop("uimg", None)
        self.sent_clouds.add(key)
        self.cloud_count += 1
        self.get_logger().info(
            f"Published pose-aware keyframe cloud {record.keyframe_uid} global_index={record.global_index} "
            f"points={int(cloud.width) * int(cloud.height)} sent_clouds={self.cloud_count} pending_clouds={len(self.pending_cloud_keys)}"
        )
        return True

    def _colors(self, frame, valid_mask):
        uimg = frame.uimg.detach().cpu().numpy() if hasattr(frame.uimg, "detach") else np.asarray(frame.uimg)
        if uimg.ndim == 4:
            uimg = uimg[0]
        colors = uimg.reshape(-1, uimg.shape[-1])[:, :3]
        colors = colors[valid_mask]
        if colors.dtype != np.uint8:
            if float(np.nanmax(colors)) <= 1.5:
                colors = colors * 255.0
            colors = np.clip(colors, 0, 255).astype(np.uint8)
        return colors

    def publish_pose_graph(self):
        if not self.dirty_poses or not self.poses:
            return
        self.pose_revision += 1
        poses = []
        for key in sorted(self.poses):
            data = self.poses[key]
            rec = self.records.get(key)
            if rec is None:
                continue
            sim3_data = [float(x) for x in data["optimized_sim3_data"]]
            poses.append({
                "robot_id": data["robot_id"],
                "kf_id": int(data["kf_id"]),
                "keyframe_uid": data.get("keyframe_uid", rec.keyframe_uid),
                "global_index": int(data.get("global_index", rec.global_index)),
                "sim3": sim3_data,
            })
        payload = {
            "schema": "multi_robot_native_unity_pose_graph_v1",
            "revision": self.pose_revision,
            "pose_count": len(poses),
            "sim3_layout": "tx_ty_tz_qx_qy_qz_qw_scale",
            "poses": poses,
        }
        msg = String(); msg.data = dumps(payload); self.pose_pub.publish(msg)
        summary = String(); summary.data = dumps({
            "schema": "multi_robot_native_unity_summary_v1",
            "revision": self.pose_revision,
            "pose_count": len(poses),
            "sent_clouds": len(self.sent_clouds),
            "pending_clouds": len(self.pending_cloud_keys),
        })
        self.summary_pub.publish(summary)
        self.get_logger().info(
            f"Published pose-aware pose graph revision={self.pose_revision} poses={len(poses)} sent_clouds={len(self.sent_clouds)} pending_clouds={len(self.pending_cloud_keys)}"
        )
        self.dirty_poses = False


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNativePoseAwareUnityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
