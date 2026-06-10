#!/usr/bin/env python3
from collections import deque
from functools import partial
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from stretch3_ros_nodes.optimized_cloud_transformer import (
    PointCloudTransformError,
    merge_transformed_clouds,
    transform_keyframe_cloud,
)
from stretch3_ros_nodes.optimized_map_store import OptimizedMapStore
from stretch3_ros_nodes.optimized_map_summary_types import (
    make_keyframe_cloud_summary,
    make_optimized_map_summary,
)
from stretch3_ros_nodes.optimized_pose_types import parse_optimized_pose
from stretch3_ros_nodes.sim3_keyframe_types import key_to_uid


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotOptimizedMapNode(Node):
    def __init__(self):
        super().__init__("multi_robot_optimized_map_node")

        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("cloud_topic_template", "/{robot_id}/mast3r/keyframe_cloud_local")
        self.declare_parameter("optimized_pose_topic", "/multi_robot/optimized_keyframe_poses")
        self.declare_parameter("optimized_keyframe_cloud_topic", "/multi_robot/optimized_keyframe_clouds")
        self.declare_parameter("optimized_keyframe_cloud_summary_topic", "/multi_robot/optimized_keyframe_cloud_summaries")
        self.declare_parameter("optimized_map_topic", "/multi_robot/optimized_map_points")
        self.declare_parameter("optimized_full_map_topic", "/multi_robot/optimized_map_points_full")
        self.declare_parameter("optimized_map_summary_topic", "/multi_robot/optimized_map_summaries")
        self.declare_parameter("output_frame", "multi_robot_optimized_map")
        self.declare_parameter("max_points_per_keyframe", 30000)
        self.declare_parameter("max_merged_points", 300000)
        self.declare_parameter("min_cloud_confidence", 0.95)
        self.declare_parameter("voxel_leaf_size_per_keyframe", 0.0)
        self.declare_parameter("voxel_leaf_size_merged", 0.0)
        self.declare_parameter("publish_merged_period_sec", 1.0)
        self.declare_parameter("publish_only_on_revision_change", True)
        self.declare_parameter("chunk_max_points", 100000)
        self.declare_parameter("chunk_publish_delay_sec", 0.333333)

        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        self.cloud_topic_template = self.get_parameter("cloud_topic_template").get_parameter_value().string_value
        pose_topic = self.get_parameter("optimized_pose_topic").get_parameter_value().string_value
        keyframe_cloud_topic = self.get_parameter("optimized_keyframe_cloud_topic").get_parameter_value().string_value
        keyframe_summary_topic = self.get_parameter("optimized_keyframe_cloud_summary_topic").get_parameter_value().string_value
        map_topic = self.get_parameter("optimized_map_topic").get_parameter_value().string_value
        full_map_topic = self.get_parameter("optimized_full_map_topic").get_parameter_value().string_value
        map_summary_topic = self.get_parameter("optimized_map_summary_topic").get_parameter_value().string_value
        self.output_frame = self.get_parameter("output_frame").get_parameter_value().string_value
        self.max_points_per_keyframe = self.get_parameter("max_points_per_keyframe").get_parameter_value().integer_value
        self.max_merged_points = self.get_parameter("max_merged_points").get_parameter_value().integer_value
        self.min_cloud_confidence = self.get_parameter("min_cloud_confidence").get_parameter_value().double_value
        self.voxel_leaf_size_per_keyframe = self.get_parameter("voxel_leaf_size_per_keyframe").get_parameter_value().double_value
        self.voxel_leaf_size_merged = self.get_parameter("voxel_leaf_size_merged").get_parameter_value().double_value
        publish_period = self.get_parameter("publish_merged_period_sec").get_parameter_value().double_value
        self.publish_only_on_revision_change = self.get_parameter(
            "publish_only_on_revision_change"
        ).get_parameter_value().bool_value
        self.chunk_max_points = max(1, self.get_parameter("chunk_max_points").get_parameter_value().integer_value)
        self.chunk_publish_delay_sec = max(0.0, self.get_parameter("chunk_publish_delay_sec").get_parameter_value().double_value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.store = OptimizedMapStore()
        self.transformed = {}
        self.key_revisions = {}
        self.map_revision = 0
        self.last_published_map_revision = -1
        self.keyframe_cloud_pub = self.create_publisher(PointCloud2, keyframe_cloud_topic, qos)
        self.keyframe_summary_pub = self.create_publisher(String, keyframe_summary_topic, qos)
        self.map_pub = self.create_publisher(PointCloud2, map_topic, qos)
        self.full_map_pub = self.create_publisher(PointCloud2, full_map_topic, qos) if full_map_topic else None
        self.full_map_topic = full_map_topic
        self.map_summary_pub = self.create_publisher(String, map_summary_topic, qos)
        self.chunk_queue = deque()
        self.pending_chunk_batch = None
        self.active_chunk_revision = None
        self._subscriptions = []
        self._subscriptions.append(self.create_subscription(String, pose_topic, self.on_optimized_pose, qos))
        self.get_logger().info(f"Listening for optimized keyframe poses on {pose_topic}")

        for robot_id in self.robot_ids:
            topic = self._format_topic(self.cloud_topic_template, robot_id, "cloud_topic_template")
            self._subscriptions.append(
                self.create_subscription(PointCloud2, topic, partial(self.on_keyframe_cloud, robot_id), qos)
            )
            self.get_logger().info(f"Listening for {robot_id} local keyframe clouds on {topic}")

        if publish_period > 0.0:
            self.timer = self.create_timer(float(publish_period), self.publish_merged_map)
        else:
            self.timer = None
        self.chunk_timer = self.create_timer(
            max(0.001, float(self.chunk_publish_delay_sec)),
            self.publish_next_map_chunk,
        )

        self.get_logger().info(
            f"Publishing optimized keyframe clouds on {keyframe_cloud_topic}; summaries on {keyframe_summary_topic}; "
            f"chunked merged map on {map_topic}; full merged map on {full_map_topic or 'disabled'}; "
            f"map summaries on {map_summary_topic}; frame={self.output_frame}; "
            f"voxel_keyframe={self.voxel_leaf_size_per_keyframe}, voxel_merged={self.voxel_leaf_size_merged}, "
            f"publish_only_on_revision_change={self.publish_only_on_revision_change}; "
            f"chunk_max_points={self.chunk_max_points}, chunk_delay={self.chunk_publish_delay_sec}"
        )

    def _format_topic(self, template: str, robot_id: str, parameter_name: str) -> str:
        try:
            return template.format(robot_id=robot_id)
        except KeyError as exc:
            raise ValueError(
                f"{parameter_name} must contain only the {{robot_id}} placeholder; got {template!r}"
            ) from exc

    def on_keyframe_cloud(self, expected_robot_id: str, msg: PointCloud2) -> None:
        try:
            key = self.store.upsert_cloud(msg)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed keyframe cloud: {exc}")
            return
        if key[0] != expected_robot_id:
            self.get_logger().warn(f"Cloud robot mismatch: topic={expected_robot_id}, frame_id={msg.header.frame_id}")
            return
        self.try_publish_key(key, reason="cloud")

    def on_optimized_pose(self, msg: String) -> None:
        try:
            pose = parse_optimized_pose(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed optimized pose: {exc}")
            return
        key = self.store.upsert_pose(pose)
        self.try_publish_key(key, reason="pose")

    def try_publish_key(self, key, reason: str) -> None:
        if not self.store.has_ready(key):
            return
        try:
            transformed = transform_keyframe_cloud(
                self.store.cloud(key),
                self.store.pose(key).transform,
                self.output_frame,
                max_points=self.max_points_per_keyframe,
                min_confidence=self.min_cloud_confidence,
                voxel_leaf_size=self.voxel_leaf_size_per_keyframe,
            )
        except PointCloudTransformError as exc:
            self.get_logger().warn(f"Failed to transform {key_to_uid(key)} after {reason}: {exc}")
            return
        self.transformed[key] = transformed
        revision = self.key_revisions.get(key, 0) + 1
        self.key_revisions[key] = revision
        self.map_revision += 1

        self.keyframe_cloud_pub.publish(transformed.msg)
        summary = String()
        summary.data = make_keyframe_cloud_summary(
            key,
            revision,
            transformed.point_count,
            self.output_frame,
            self.voxel_leaf_size_per_keyframe,
            reason,
        )
        self.keyframe_summary_pub.publish(summary)
        self.get_logger().info(
            f"Published optimized cloud {key_to_uid(key)} revision={revision} points={transformed.point_count}; "
            f"ready={len(self.transformed)} map_revision={self.map_revision}"
        )

    def publish_merged_map(self) -> None:
        if not self.transformed:
            return
        if self.chunk_queue:
            return
        if self.publish_only_on_revision_change and self.map_revision == self.last_published_map_revision:
            return
        msg = merge_transformed_clouds(
            list(self.transformed.values()),
            self.output_frame,
            self.get_clock().now().to_msg(),
            max_points=self.max_merged_points,
            voxel_leaf_size=self.voxel_leaf_size_merged,
        )
        if msg is None:
            return
        point_count = int(msg.width) * int(msg.height)
        revision = self.map_revision
        if self.full_map_pub is not None:
            self.full_map_pub.publish(msg)
        chunk_count = self.queue_map_chunks(msg, revision)
        self.last_published_map_revision = revision
        summary = String()
        payload = json.loads(make_optimized_map_summary(
            revision,
            len(self.transformed),
            point_count,
            self.output_frame,
            self.max_merged_points,
            self.voxel_leaf_size_merged,
        ))
        payload.update({
            "chunked": True,
            "full_map_topic": self.full_map_topic,
            "chunk_count": int(chunk_count),
            "chunk_max_points": int(self.chunk_max_points),
            "chunk_publish_delay_sec": float(self.chunk_publish_delay_sec),
        })
        summary.data = json.dumps(payload, separators=(",", ":"))
        self.map_summary_pub.publish(summary)
        self.get_logger().info(
            f"Queued optimized merged map revision={revision} keyframes={len(self.transformed)} "
            f"points={point_count} chunks={chunk_count} chunk_max_points={self.chunk_max_points}"
        )

    def queue_map_chunks(self, msg: PointCloud2, revision: int) -> int:
        chunks = self.make_map_chunks(msg, revision)
        if not self.chunk_queue:
            self.chunk_queue.extend(chunks)
            self.active_chunk_revision = revision
            self.get_logger().info(f"Started paced map chunk stream revision={revision} chunks={len(chunks)}")
        else:
            self.pending_chunk_batch = (revision, chunks)
            self.get_logger().info(
                f"Deferred map revision={revision} chunks={len(chunks)} until active revision="
                f"{self.active_chunk_revision} finishes"
            )
        return len(chunks)

    def make_map_chunks(self, msg: PointCloud2, revision: int) -> list[PointCloud2]:
        point_count = int(msg.width) * int(msg.height)
        if point_count <= 0:
            return []
        chunk_count = max(1, math.ceil(point_count / self.chunk_max_points))
        if chunk_count == 1:
            msg.header.frame_id = self.output_frame
            return [msg]

        data = bytes(msg.data)
        point_step = int(msg.point_step)
        stamp = self.get_clock().now().to_msg()
        chunks = []
        for chunk_index in range(chunk_count):
            start = chunk_index * self.chunk_max_points
            end = min(start + self.chunk_max_points, point_count)
            chunk = PointCloud2()
            chunk.header.stamp = stamp
            chunk.header.frame_id = f"kf_999999_{revision}_{chunk_index + 1}_{chunk_count}"
            chunk.height = 1
            chunk.width = int(end - start)
            chunk.fields = msg.fields
            chunk.is_bigendian = msg.is_bigendian
            chunk.point_step = msg.point_step
            chunk.row_step = int(chunk.point_step) * int(chunk.width)
            chunk.is_dense = msg.is_dense
            byte_start = start * point_step
            byte_end = end * point_step
            chunk.data = data[byte_start:byte_end]
            chunks.append(chunk)
        return chunks

    def publish_next_map_chunk(self) -> None:
        if self.chunk_queue:
            chunk = self.chunk_queue.popleft()
            self.map_pub.publish(chunk)
            if not self.chunk_queue:
                self.get_logger().info(f"Finished paced map chunk stream revision={self.active_chunk_revision}")
                if self.pending_chunk_batch is not None:
                    revision, chunks = self.pending_chunk_batch
                    self.pending_chunk_batch = None
                    self.chunk_queue.extend(chunks)
                    self.active_chunk_revision = revision
                    self.get_logger().info(
                        f"Started pending paced map chunk stream revision={revision} chunks={len(chunks)}"
                    )
                else:
                    self.active_chunk_revision = None


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotOptimizedMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
