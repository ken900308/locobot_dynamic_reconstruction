#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stretch3_ros_nodes.pgo_publishers import (
    make_optimized_pose_json,
    make_robot_alignments_json,
    make_solution_summary,
)
from stretch3_ros_nodes.pgo_solver import Sim3AlignmentGraph
from stretch3_ros_nodes.pgo_types import parse_pose_constraint
from stretch3_ros_nodes.sim3_keyframe_types import parse_keyframe_metadata


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotPgoNode(Node):
    def __init__(self):
        super().__init__("multi_robot_pgo_node")

        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("anchor_robot", "")
        self.declare_parameter("metadata_topic_template", "/{robot_id}/mast3r/keyframe_metadata")
        self.declare_parameter("pose_constraint_topic", "/multi_robot/pose_constraints")
        self.declare_parameter("robot_alignments_topic", "/multi_robot/robot_alignments")
        self.declare_parameter("optimized_pose_topic", "/multi_robot/optimized_keyframe_poses")
        self.declare_parameter("optimized_pose_topic_template", "/{robot_id}/mast3r/keyframe_pose_optimized")
        self.declare_parameter("pgo_summary_topic", "/multi_robot/pgo_summaries")
        self.declare_parameter("min_constraint_confidence", 0.0)
        self.declare_parameter("max_constraint_rmse_m", 0.20)
        self.declare_parameter("min_constraint_inliers", 12)
        self.declare_parameter("max_alignment_translation_residual_m", 1.0)
        self.declare_parameter("max_alignment_rotation_residual_deg", 30.0)
        self.declare_parameter("max_alignment_log_scale_residual", 0.25)
        self.declare_parameter("min_pair_observations_for_robust", 3)
        self.declare_parameter("publish_all_on_update", True)

        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        anchor_param = self.get_parameter("anchor_robot").get_parameter_value().string_value.strip()
        self.anchor_robot = anchor_param or self.robot_ids[0]
        self.metadata_topic_template = self.get_parameter("metadata_topic_template").get_parameter_value().string_value
        constraint_topic = self.get_parameter("pose_constraint_topic").get_parameter_value().string_value
        alignments_topic = self.get_parameter("robot_alignments_topic").get_parameter_value().string_value
        optimized_topic = self.get_parameter("optimized_pose_topic").get_parameter_value().string_value
        optimized_template = self.get_parameter("optimized_pose_topic_template").get_parameter_value().string_value
        summary_topic = self.get_parameter("pgo_summary_topic").get_parameter_value().string_value
        self.min_constraint_confidence = self.get_parameter("min_constraint_confidence").get_parameter_value().double_value
        self.max_constraint_rmse_m = self.get_parameter("max_constraint_rmse_m").get_parameter_value().double_value
        self.min_constraint_inliers = self.get_parameter("min_constraint_inliers").get_parameter_value().integer_value
        self.max_alignment_translation_residual_m = self.get_parameter(
            "max_alignment_translation_residual_m"
        ).get_parameter_value().double_value
        self.max_alignment_rotation_residual_deg = self.get_parameter(
            "max_alignment_rotation_residual_deg"
        ).get_parameter_value().double_value
        self.max_alignment_log_scale_residual = self.get_parameter(
            "max_alignment_log_scale_residual"
        ).get_parameter_value().double_value
        self.min_pair_observations_for_robust = self.get_parameter(
            "min_pair_observations_for_robust"
        ).get_parameter_value().integer_value
        self.publish_all_on_update = self.get_parameter("publish_all_on_update").get_parameter_value().bool_value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.graph = Sim3AlignmentGraph(anchor_robot=self.anchor_robot)
        self.alignments_pub = self.create_publisher(String, alignments_topic, qos)
        self.optimized_pub = self.create_publisher(String, optimized_topic, qos)
        self.summary_pub = self.create_publisher(String, summary_topic, qos)
        self.per_robot_pose_pubs = {}
        self._subscriptions = []

        self._subscriptions.append(self.create_subscription(String, constraint_topic, self.on_pose_constraint, qos))
        self.get_logger().info(f"Listening for Sim(3) pose constraints on {constraint_topic}")

        for robot_id in self.robot_ids:
            topic = self._format_topic(self.metadata_topic_template, robot_id, "metadata_topic_template")
            output_topic = self._format_topic(optimized_template, robot_id, "optimized_pose_topic_template")
            self._subscriptions.append(
                self.create_subscription(String, topic, partial(self.on_keyframe_metadata, robot_id), qos)
            )
            self.per_robot_pose_pubs[robot_id] = self.create_publisher(String, output_topic, qos)
            self.get_logger().info(f"Listening for {robot_id} keyframe metadata on {topic}")
            self.get_logger().info(f"Publishing {robot_id} optimized poses on {output_topic}")

        self.get_logger().info(
            f"Publishing robot alignments on {alignments_topic}; optimized poses on {optimized_topic}; "
            f"summary on {summary_topic}; anchor={self.anchor_robot}; "
            f"robust_translation={self.max_alignment_translation_residual_m}m, "
            f"robust_rotation={self.max_alignment_rotation_residual_deg}deg"
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
        self.graph.upsert_keyframe(keyframe)
        self.solve_and_publish(reason=f"metadata {keyframe.keyframe_uid}")

    def on_pose_constraint(self, msg: String) -> None:
        try:
            constraint = parse_pose_constraint(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed pose constraint: {exc}")
            return
        self.graph.upsert_constraint(constraint)
        self.solve_and_publish(reason=f"constraint {constraint.from_uid}->{constraint.to_uid}")

    def solve_and_publish(self, reason: str) -> None:
        try:
            solution = self.graph.solve(
                min_confidence=self.min_constraint_confidence,
                max_rmse_m=self.max_constraint_rmse_m,
                min_inliers=self.min_constraint_inliers,
                max_alignment_translation_residual_m=self.max_alignment_translation_residual_m,
                max_alignment_rotation_residual_deg=self.max_alignment_rotation_residual_deg,
                max_alignment_log_scale_residual=self.max_alignment_log_scale_residual,
                min_pair_observations_for_robust=self.min_pair_observations_for_robust,
            )
        except Exception as exc:
            self.get_logger().warn(f"PGO solve failed after {reason}: {exc}")
            return

        alignment_msg = String()
        alignment_msg.data = make_robot_alignments_json(solution)
        self.alignments_pub.publish(alignment_msg)

        summary = make_solution_summary(solution)
        summary_msg = String()
        summary_msg.data = summary
        self.summary_pub.publish(summary_msg)
        self.get_logger().info(f"PGO update after {reason}: {summary}")

        if self.publish_all_on_update:
            keys = sorted(solution.optimized_keyframes.keys())
        else:
            keys = []
        for key in keys:
            self.publish_optimized_pose(key, solution)

    def publish_optimized_pose(self, key, solution) -> None:
        transform = solution.optimized_keyframes[key]
        msg = String()
        msg.data = make_optimized_pose_json(key, transform, solution)
        self.optimized_pub.publish(msg)
        robot_pub = self.per_robot_pose_pubs.get(key[0])
        if robot_pub is not None:
            robot_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPgoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
