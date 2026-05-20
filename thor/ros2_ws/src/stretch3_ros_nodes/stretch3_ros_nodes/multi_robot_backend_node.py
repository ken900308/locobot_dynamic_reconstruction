#!/usr/bin/env python3
import json
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from stretch3_ros_nodes.descriptor_matcher import DescriptorMatcher
from stretch3_ros_nodes.keyframe_cloud_store import KeyframeCloudStore
from stretch3_ros_nodes.keyframe_store import KeyframeStore
from stretch3_ros_nodes.sim3_keyframe_types import LoopCandidate, key_to_uid, parse_keyframe_metadata


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotBackendNode(Node):
    def __init__(self):
        super().__init__("multi_robot_backend_node")

        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("metadata_topic_template", "/{robot_id}/mast3r/keyframe_metadata")
        self.declare_parameter("cloud_topic_template", "/{robot_id}/mast3r/keyframe_cloud_local")
        self.declare_parameter("candidate_topic", "/multi_robot/loop_candidates")
        self.declare_parameter("candidate_summary_topic", "/multi_robot/loop_candidate_summaries")
        self.declare_parameter("verification_job_topic", "/multi_robot/geometric_verification_jobs")
        self.declare_parameter("min_similarity", 0.82)
        self.declare_parameter("top_k", 3)

        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        self.metadata_topic_template = self.get_parameter("metadata_topic_template").get_parameter_value().string_value
        self.cloud_topic_template = self.get_parameter("cloud_topic_template").get_parameter_value().string_value
        candidate_topic = self.get_parameter("candidate_topic").get_parameter_value().string_value
        summary_topic = self.get_parameter("candidate_summary_topic").get_parameter_value().string_value
        verification_job_topic = self.get_parameter("verification_job_topic").get_parameter_value().string_value
        min_similarity = self.get_parameter("min_similarity").get_parameter_value().double_value
        top_k = self.get_parameter("top_k").get_parameter_value().integer_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.store = KeyframeStore()
        self.cloud_store = KeyframeCloudStore()
        self.matcher = DescriptorMatcher(min_similarity=min_similarity, top_k=top_k)
        self.candidate_pub = self.create_publisher(String, candidate_topic, qos)
        self.summary_pub = self.create_publisher(String, summary_topic, qos)
        self.verification_job_pub = self.create_publisher(String, verification_job_topic, qos)
        self._subscriptions = []
        self._pending_candidates: dict[tuple, LoopCandidate] = {}

        for robot_id in self.robot_ids:
            metadata_topic = self._format_topic(self.metadata_topic_template, robot_id, "metadata_topic_template")
            cloud_topic = self._format_topic(self.cloud_topic_template, robot_id, "cloud_topic_template")
            self._subscriptions.append(
                self.create_subscription(
                    String,
                    metadata_topic,
                    partial(self.on_keyframe_metadata, robot_id),
                    qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    cloud_topic,
                    self.on_keyframe_cloud,
                    qos,
                )
            )
            self.get_logger().info(f"Listening for {robot_id} keyframes on {metadata_topic}")
            self.get_logger().info(f"Listening for {robot_id} local clouds on {cloud_topic}")

        self.get_logger().info(
            f"Publishing descriptor loop candidates on {candidate_topic}; "
            f"summaries on {summary_topic}; verification jobs on {verification_job_topic}; "
            f"min_similarity={min_similarity}, top_k={top_k}"
        )

    def _format_topic(self, template: str, robot_id: str, parameter_name: str) -> str:
        try:
            return template.format(robot_id=robot_id)
        except KeyError as exc:
            raise ValueError(
                f"{parameter_name} must contain only the {{robot_id}} placeholder; got {template!r}"
            ) from exc

    def on_keyframe_metadata(self, expected_robot_id: str, msg: String) -> None:
        try:
            keyframe = parse_keyframe_metadata(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed keyframe metadata: {exc}")
            return

        if keyframe.robot_id != expected_robot_id:
            self.get_logger().warn(
                f"Metadata robot_id mismatch: topic={expected_robot_id}, payload={keyframe.robot_id}"
            )
            return

        is_new = self.store.upsert(keyframe)
        candidates = self.matcher.find_candidates(
            keyframe,
            self.store.by_other_robots(keyframe.robot_id),
        )

        for candidate in candidates:
            self.publish_candidate(candidate)

        if is_new or candidates:
            self.get_logger().info(
                f"Stored {keyframe.keyframe_uid}; total={self.store.count()}, "
                f"clouds={self.cloud_store.count()}, robots={self.store.robot_counts()}, "
                f"candidates={len(candidates)}"
            )

    def on_keyframe_cloud(self, msg: PointCloud2) -> None:
        try:
            record = self.cloud_store.upsert(msg)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed keyframe cloud: {exc}")
            return

        self.get_logger().info(
            f"Stored local cloud {record.frame_id}; points={record.point_count}, "
            f"fields={record.fields}, clouds={self.cloud_store.count()}"
        )
        self.flush_ready_verification_jobs()

    def publish_candidate(self, candidate: LoopCandidate) -> None:
        verification_ready = self.is_verification_ready(candidate)

        candidate_msg = String()
        candidate_msg.data = candidate.to_json()
        self.candidate_pub.publish(candidate_msg)

        self.publish_candidate_summary(candidate, verification_ready)

        if verification_ready:
            self.publish_verification_job(candidate)
        else:
            self._pending_candidates[self.candidate_key(candidate)] = candidate


    def publish_candidate_summary(self, candidate: LoopCandidate, verification_ready: bool) -> None:
        summary_msg = String()
        summary_msg.data = candidate.to_summary(verification_ready)
        self.summary_pub.publish(summary_msg)
        self.get_logger().info(summary_msg.data)

    def candidate_key(self, candidate: LoopCandidate) -> tuple:
        return (candidate.from_key, candidate.to_key)

    def flush_ready_verification_jobs(self) -> None:
        ready_keys = [
            key for key, candidate in self._pending_candidates.items()
            if self.is_verification_ready(candidate)
        ]
        for key in ready_keys:
            candidate = self._pending_candidates.pop(key)
            self.publish_verification_job(candidate)

    def publish_verification_job(self, candidate: LoopCandidate) -> None:
        job_msg = String()
        job_msg.data = self.make_verification_job(candidate)
        self.verification_job_pub.publish(job_msg)
        self.publish_candidate_summary(candidate, True)
        self.get_logger().info(
            f"Queued geometric verification job: {key_to_uid(candidate.from_key)} -> "
            f"{key_to_uid(candidate.to_key)} similarity={candidate.similarity:.4f}"
        )

    def is_verification_ready(self, candidate: LoopCandidate) -> bool:
        return (
            self.store.get(candidate.from_key) is not None
            and self.store.get(candidate.to_key) is not None
            and self.cloud_store.has(candidate.from_key)
            and self.cloud_store.has(candidate.to_key)
        )

    def make_verification_job(self, candidate: LoopCandidate) -> str:
        from_kf = self.store.get(candidate.from_key)
        to_kf = self.store.get(candidate.to_key)
        from_cloud = self.cloud_store.get(candidate.from_key)
        to_cloud = self.cloud_store.get(candidate.to_key)
        if from_kf is None or to_kf is None or from_cloud is None or to_cloud is None:
            raise ValueError("verification job requested before metadata/clouds were ready")

        payload = {
            "schema": "geometric_verification_job_v1",
            "candidate": candidate.to_dict(),
            "from_uid": key_to_uid(candidate.from_key),
            "to_uid": key_to_uid(candidate.to_key),
            "from_sim3_data": from_kf.sim3_data,
            "to_sim3_data": to_kf.sim3_data,
            "sim3_layout": "lietorch_sim3_data_tx_ty_tz_qx_qy_qz_qw_s",
            "from_cloud": {
                "frame_id": from_cloud.frame_id,
                "point_count": from_cloud.point_count,
                "fields": list(from_cloud.fields),
            },
            "to_cloud": {
                "frame_id": to_cloud.frame_id,
                "point_count": to_cloud.point_count,
                "fields": list(to_cloud.fields),
            },
        }
        return json.dumps(payload, separators=(",", ":"))


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
