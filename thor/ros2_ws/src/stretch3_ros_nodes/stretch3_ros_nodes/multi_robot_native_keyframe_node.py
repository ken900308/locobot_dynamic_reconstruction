#!/usr/bin/env python3
from functools import partial
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_keyframe_index import CrossRobotKeyframeIndex
from stretch3_ros_nodes.cross_robot_native_cache import (
    load_native_keyframe_cache,
    manifest_from_metadata_json,
    validate_native_cache_manifest,
)


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotNativeKeyframeNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_keyframe_node")
        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("metadata_topic_template", "/{robot_id}/mast3r/keyframe_metadata")
        self.declare_parameter("summary_topic", "/multi_robot/native_keyframe_summaries")
        self.declare_parameter("load_cache", False)

        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        self.metadata_topic_template = self.get_parameter("metadata_topic_template").get_parameter_value().string_value
        summary_topic = self.get_parameter("summary_topic").get_parameter_value().string_value
        self.load_cache = self.get_parameter("load_cache").get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.index = CrossRobotKeyframeIndex()
        self.summary_pub = self.create_publisher(String, summary_topic, qos)
        self._subscriptions = []
        for robot_id in self.robot_ids:
            topic = self.metadata_topic_template.format(robot_id=robot_id)
            self._subscriptions.append(self.create_subscription(String, topic, partial(self.on_metadata, robot_id), qos))
            self.get_logger().info(f"Listening for native-cache metadata from {robot_id} on {topic}")
        self.get_logger().info(f"Publishing native keyframe summaries on {summary_topic}; load_cache={self.load_cache}")

    def on_metadata(self, expected_robot_id: str, msg: String) -> None:
        try:
            manifest = manifest_from_metadata_json(msg.data)
            if manifest.robot_id != expected_robot_id:
                raise ValueError(f"robot_id mismatch: topic={expected_robot_id}, payload={manifest.robot_id}")
            validate_native_cache_manifest(manifest)
            loaded_shapes = None
            if self.load_cache:
                record = load_native_keyframe_cache(manifest)
                loaded_shapes = {
                    name: list(value.shape)
                    for name, value in record.payload.items()
                    if hasattr(value, "shape")
                }
        except Exception as exc:
            self.get_logger().warn(f"Rejected native keyframe metadata: {exc}")
            return

        ref = self.index.upsert(manifest.robot_id, manifest.kf_id, manifest.keyframe_uid, manifest.cache_path)
        payload = {
            "schema": "multi_robot_native_keyframe_summary_v1",
            "robot_id": manifest.robot_id,
            "kf_id": manifest.kf_id,
            "keyframe_uid": manifest.keyframe_uid,
            "global_index": ref.global_index,
            "cache_path": manifest.cache_path,
            "cache_schema": manifest.schema,
            "tensor_shapes": manifest.tensor_shapes,
            "loaded_tensor_shapes": loaded_shapes,
            "total_keyframes": len(self.index),
            "robot_counts": self.index.robot_counts(),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.summary_pub.publish(out)
        self.get_logger().info(
            f"Indexed native keyframe {manifest.keyframe_uid} as global_idx={ref.global_index}; "
            f"total={len(self.index)}, robots={self.index.robot_counts()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotNativeKeyframeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
