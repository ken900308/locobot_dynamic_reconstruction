#!/usr/bin/env python3
from functools import partial
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_keyframe_index import CrossRobotKeyframeIndex
from stretch3_ros_nodes.cross_robot_native_cache import manifest_from_metadata_json, validate_native_cache_manifest
from stretch3_ros_nodes.native_frame_cache import load_frame_record, record_to_frame
from stretch3_ros_nodes.native_imports import configure_mast3r_imports
from stretch3_ros_nodes.native_messages import dumps


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotNativeRetrievalNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_retrieval_node")
        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("metadata_topic_template", "/{robot_id}/mast3r/keyframe_metadata")
        self.declare_parameter("candidate_topic", "/multi_robot/native_retrieval_candidates")
        self.declare_parameter("summary_topic", "/multi_robot/native_retrieval_summaries")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("mast3r_config_path", "config/base.yaml")
        self.declare_parameter("mast3r_model_path", "")
        self.declare_parameter("mast3r_retriever_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("top_k", 3)
        self.declare_parameter("min_thresh", 0.005)
        self.declare_parameter("query_k_multiplier", 4)

        self.robot_ids = _parse_csv(self.get_parameter("robot_ids").value) or ["robot1", "robot2"]
        self.metadata_topic_template = self.get_parameter("metadata_topic_template").value
        self.device = self.get_parameter("device").value
        self.top_k = int(self.get_parameter("top_k").value)
        self.min_thresh = float(self.get_parameter("min_thresh").value)
        self.query_k_multiplier = max(1, int(self.get_parameter("query_k_multiplier").value))
        self.robot_order = {robot_id: i for i, robot_id in enumerate(self.robot_ids)}
        self.mast3r_slam_root = self.get_parameter("mast3r_slam_root").value
        self.mast3r_config_path = self.get_parameter("mast3r_config_path").value
        self.mast3r_model_path = self.get_parameter("mast3r_model_path").value.strip() or None
        self.mast3r_retriever_path = self.get_parameter("mast3r_retriever_path").value.strip() or None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.index = CrossRobotKeyframeIndex()
        self.records_by_db_id = {}
        self.retriever = None
        self.model = None
        self.candidate_pub = self.create_publisher(String, self.get_parameter("candidate_topic").value, qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("summary_topic").value, qos)
        self._metadata_subscriptions = []
        for robot_id in self.robot_ids:
            topic = self.metadata_topic_template.format(robot_id=robot_id)
            self._metadata_subscriptions.append(self.create_subscription(String, topic, partial(self.on_metadata, robot_id), qos))
            self.get_logger().info(f"Listening for native retrieval metadata from {robot_id} on {topic}")
        self.get_logger().info(
            f"Native retrieval ready: root={self.mast3r_slam_root}, device={self.device}, top_k={self.top_k}, min_thresh={self.min_thresh}"
        )

    def _ensure_loaded(self):
        if self.retriever is not None:
            return
        started = time.monotonic()
        root = configure_mast3r_imports(self.mast3r_slam_root)
        from mast3r_slam.config import load_config
        from mast3r_slam.mast3r_utils import load_mast3r, load_retriever
        load_config(str(root / self.mast3r_config_path))
        model_path = self.mast3r_model_path or str(root / "checkpoints" / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")
        retriever_path = self.mast3r_retriever_path or str(root / "checkpoints" / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth")
        self.model = load_mast3r(model_path, device=self.device)
        self.retriever = load_retriever(self.model, retriever_path, device=self.device)
        self.get_logger().info(f"Loaded MASt3R model/retriever in {time.monotonic() - started:.2f}s")

    def on_metadata(self, expected_robot_id: str, msg: String) -> None:
        try:
            self._ensure_loaded()
            manifest = manifest_from_metadata_json(msg.data)
            if manifest.robot_id != expected_robot_id:
                raise ValueError(f"robot mismatch topic={expected_robot_id} payload={manifest.robot_id}")
            validate_native_cache_manifest(manifest)
            ref = self.index.upsert(manifest.robot_id, manifest.kf_id, manifest.keyframe_uid, manifest.cache_path)
            # Native FactorGraph pins the lowest sorted keyframe index. Use deterministic
            # robot-order offsets so anchor robot keyframes sort before other robots.
            global_index = self.robot_order.get(manifest.robot_id, len(self.robot_order)) * 1000000 + int(manifest.kf_id)
            record = load_frame_record(manifest, global_index, self.device)
            frame = record_to_frame(record)
        except Exception as exc:
            self.get_logger().warn(f"Rejected native retrieval keyframe: {exc}")
            return

        query_k = max(self.top_k * self.query_k_multiplier, self.top_k)
        database_size = int(self.retriever.kf_counter)
        try:
            inds = self.retriever.update(frame, add_after_query=False, k=query_k, min_thresh=self.min_thresh)
        except Exception as exc:
            self.get_logger().warn(f"Retrieval failed for {manifest.keyframe_uid}: {exc}")
            return

        candidates = []
        for db_id in inds:
            other = self.records_by_db_id.get(int(db_id))
            if other is None or other.robot_id == record.robot_id:
                continue
            candidates.append(other)
            if len(candidates) >= self.top_k:
                break
        # Add after querying, mirroring native MASt3R-SLAM retrieval semantics.
        try:
            self.retriever.update(frame, add_after_query=True, k=query_k, min_thresh=self.min_thresh)
            self.records_by_db_id[int(self.retriever.kf_counter) - 1] = record
        except Exception as exc:
            self.get_logger().warn(f"Failed to add {manifest.keyframe_uid} to retrieval DB: {exc}")
            return

        for rank, other in enumerate(candidates):
            payload = {
                "schema": "multi_robot_native_retrieval_candidate_v1",
                "query_robot": record.robot_id,
                "query_kf_id": record.kf_id,
                "query_uid": record.keyframe_uid,
                "query_global_idx": record.global_index,
                "query_cache_path": record.cache_path,
                "match_robot": other.robot_id,
                "match_kf_id": other.kf_id,
                "match_uid": other.keyframe_uid,
                "match_global_idx": other.global_index,
                "match_cache_path": other.cache_path,
                "rank": rank,
                "top_k": self.top_k,
                "min_thresh": self.min_thresh,
                "candidate_source": "mast3r_retrieval_database",
            }
            out = String(); out.data = dumps(payload); self.candidate_pub.publish(out)

        summary = {
            "schema": "multi_robot_native_retrieval_summary_v1",
            "query_uid": record.keyframe_uid,
            "database_size": database_size,
            "top_k": self.top_k,
            "min_thresh": self.min_thresh,
            "candidate_count": len(candidates),
            "total_indexed": len(self.index),
            "robot_counts": self.index.robot_counts(),
        }
        out = String(); out.data = dumps(summary); self.summary_pub.publish(out)
        self.get_logger().info(
            f"Native retrieval query={record.keyframe_uid} database_size={database_size} top_k={self.top_k} "
            f"min_thresh={self.min_thresh} candidate_count={len(candidates)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNativeRetrievalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
