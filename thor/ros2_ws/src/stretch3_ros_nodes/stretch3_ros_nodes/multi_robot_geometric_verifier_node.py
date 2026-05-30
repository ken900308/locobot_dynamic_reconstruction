#!/usr/bin/env python3
from functools import partial
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from stretch3_ros_nodes.geometric_constraint_estimator import (
    GeometricVerifierConfig,
    estimate_geometric_constraint,
)
from stretch3_ros_nodes.keyframe_cloud_store import KeyframeCloudStore
from stretch3_ros_nodes.mast3r_symmetric_constraint_estimator import (
    Mast3rSymmetricConstraintEstimator,
    Mast3rSymmetricVerifierConfig,
)
from stretch3_ros_nodes.keyframe_image_store import KeyframeImageStore
from stretch3_ros_nodes.pointcloud_summary import PointCloudSummaryError, summarize_cloud
from stretch3_ros_nodes.pose_constraint_types import make_pose_constraint_json
from stretch3_ros_nodes.verification_job_types import GeometricVerificationJob, make_result_json, parse_verification_job


def _parse_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


class MultiRobotGeometricVerifierNode(Node):
    def __init__(self):
        super().__init__("multi_robot_geometric_verifier_node")

        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("cloud_topic_template", "/{robot_id}/mast3r/keyframe_cloud_local")
        self.declare_parameter("image_topic_template", "/{robot_id}/mast3r/keyframe_image")
        self.declare_parameter("verification_job_topic", "/multi_robot/geometric_verification_jobs")
        self.declare_parameter("verification_result_topic", "/multi_robot/geometric_verification_results")
        self.declare_parameter("verification_summary_topic", "/multi_robot/geometric_verification_summaries")
        self.declare_parameter("pose_constraint_topic", "/multi_robot/pose_constraints")
        self.declare_parameter("min_points", 2000)
        self.declare_parameter("max_summary_points", 20000)
        self.declare_parameter("max_features", 1500)
        self.declare_parameter("max_feature_matches", 300)
        self.declare_parameter("feature_ratio", 0.75)
        self.declare_parameter("max_correspondence_px", 6)
        self.declare_parameter("min_3d_correspondences", 18)
        self.declare_parameter("min_inliers", 12)
        self.declare_parameter("ransac_iterations", 160)
        self.declare_parameter("ransac_inlier_threshold_m", 0.20)
        self.declare_parameter("max_rmse_m", 0.18)
        self.declare_parameter("min_inlier_ratio", 0.55)
        self.declare_parameter("min_cloud_confidence", 0.0)
        self.declare_parameter("verifier_backend", "mast3r_symmetric")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("mast3r_model_path", "")
        self.declare_parameter("mast3r_config_path", "config/base.yaml")
        self.declare_parameter("mast3r_device", "cuda:0")
        self.declare_parameter("mast3r_image_size", 512)
        self.declare_parameter("mast3r_q_conf", 1.5)
        self.declare_parameter("mast3r_max_matches", 600)

        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        self.cloud_topic_template = self.get_parameter("cloud_topic_template").get_parameter_value().string_value
        self.image_topic_template = self.get_parameter("image_topic_template").get_parameter_value().string_value
        job_topic = self.get_parameter("verification_job_topic").get_parameter_value().string_value
        result_topic = self.get_parameter("verification_result_topic").get_parameter_value().string_value
        summary_topic = self.get_parameter("verification_summary_topic").get_parameter_value().string_value
        pose_constraint_topic = self.get_parameter("pose_constraint_topic").get_parameter_value().string_value
        self.verifier_backend = self.get_parameter("verifier_backend").get_parameter_value().string_value.strip().lower()
        self.min_points = self.get_parameter("min_points").get_parameter_value().integer_value
        self.max_summary_points = self.get_parameter("max_summary_points").get_parameter_value().integer_value
        max_correspondence_px = self.get_parameter("max_correspondence_px").get_parameter_value().integer_value
        min_3d_correspondences = self.get_parameter("min_3d_correspondences").get_parameter_value().integer_value
        min_inliers = self.get_parameter("min_inliers").get_parameter_value().integer_value
        ransac_iterations = self.get_parameter("ransac_iterations").get_parameter_value().integer_value
        ransac_inlier_threshold_m = self.get_parameter("ransac_inlier_threshold_m").get_parameter_value().double_value
        max_rmse_m = self.get_parameter("max_rmse_m").get_parameter_value().double_value
        min_inlier_ratio = self.get_parameter("min_inlier_ratio").get_parameter_value().double_value
        min_cloud_confidence = self.get_parameter("min_cloud_confidence").get_parameter_value().double_value
        self.config = GeometricVerifierConfig(
            max_features=self.get_parameter("max_features").get_parameter_value().integer_value,
            max_feature_matches=self.get_parameter("max_feature_matches").get_parameter_value().integer_value,
            feature_ratio=self.get_parameter("feature_ratio").get_parameter_value().double_value,
            max_correspondence_px=max_correspondence_px,
            min_3d_correspondences=min_3d_correspondences,
            min_inliers=min_inliers,
            ransac_iterations=ransac_iterations,
            ransac_inlier_threshold_m=ransac_inlier_threshold_m,
            max_rmse_m=max_rmse_m,
            min_inlier_ratio=min_inlier_ratio,
            min_cloud_confidence=min_cloud_confidence,
        )
        self.mast3r_estimator = None
        if self.verifier_backend == "mast3r_symmetric":
            mast3r_config = Mast3rSymmetricVerifierConfig(
                mast3r_slam_root=self.get_parameter("mast3r_slam_root").get_parameter_value().string_value,
                mast3r_model_path=self.get_parameter("mast3r_model_path").get_parameter_value().string_value,
                mast3r_config_path=self.get_parameter("mast3r_config_path").get_parameter_value().string_value,
                mast3r_device=self.get_parameter("mast3r_device").get_parameter_value().string_value,
                mast3r_image_size=self.get_parameter("mast3r_image_size").get_parameter_value().integer_value,
                mast3r_q_conf=self.get_parameter("mast3r_q_conf").get_parameter_value().double_value,
                mast3r_max_matches=self.get_parameter("mast3r_max_matches").get_parameter_value().integer_value,
                max_correspondence_px=max_correspondence_px,
                min_3d_correspondences=min_3d_correspondences,
                min_inliers=min_inliers,
                ransac_iterations=ransac_iterations,
                ransac_inlier_threshold_m=ransac_inlier_threshold_m,
                max_rmse_m=max_rmse_m,
                min_inlier_ratio=min_inlier_ratio,
                min_cloud_confidence=min_cloud_confidence,
            )
            self.mast3r_estimator = Mast3rSymmetricConstraintEstimator(mast3r_config, self.get_logger())
        elif self.verifier_backend != "orb":
            raise ValueError(f"unsupported verifier_backend: {self.verifier_backend}")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.cloud_store = KeyframeCloudStore()
        self.image_store = KeyframeImageStore()
        self.pending_jobs: dict[tuple, GeometricVerificationJob] = {}
        self.result_pub = self.create_publisher(String, result_topic, qos)
        self.summary_pub = self.create_publisher(String, summary_topic, qos)
        self.pose_constraint_pub = self.create_publisher(String, pose_constraint_topic, qos)
        self._subscriptions = []

        self._subscriptions.append(self.create_subscription(String, job_topic, self.on_verification_job, qos))
        self.get_logger().info(f"Listening for verification jobs on {job_topic}")

        for robot_id in self.robot_ids:
            cloud_topic = self._format_topic(self.cloud_topic_template, robot_id, "cloud_topic_template")
            image_topic = self._format_topic(self.image_topic_template, robot_id, "image_topic_template")
            self._subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    cloud_topic,
                    partial(self.on_keyframe_cloud, robot_id),
                    qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    image_topic,
                    partial(self.on_keyframe_image, robot_id),
                    qos,
                )
            )
            self.get_logger().info(f"Listening for {robot_id} local clouds on {cloud_topic}")
            self.get_logger().info(f"Listening for {robot_id} keyframe images on {image_topic}")

        self.get_logger().info(
            f"Publishing geometric verification results on {result_topic}; summaries on {summary_topic}; "
            f"pose constraints on {pose_constraint_topic}; min_points={self.min_points}; "
            f"verifier_backend={self.verifier_backend}"
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
            record = self.cloud_store.upsert(msg)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed keyframe cloud: {exc}")
            return
        if record.key[0] != expected_robot_id:
            self.get_logger().warn(
                f"Cloud robot_id mismatch: topic={expected_robot_id}, frame_id={record.frame_id}"
            )
            return
        self.flush_pending_jobs()

    def on_keyframe_image(self, expected_robot_id: str, msg: Image) -> None:
        try:
            record = self.image_store.upsert(msg)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed keyframe image: {exc}")
            return
        if record.key[0] != expected_robot_id:
            self.get_logger().warn(
                f"Image robot_id mismatch: topic={expected_robot_id}, frame_id={record.frame_id}"
            )
            return
        self.flush_pending_jobs()

    def on_verification_job(self, msg: String) -> None:
        try:
            job = parse_verification_job(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Rejected malformed verification job: {exc}")
            return

        self.get_logger().info(
            f"Received verification job: {job.from_uid} -> {job.to_uid} "
            f"similarity={float(job.candidate.get('similarity', 0.0)):.4f}"
        )
        if not self.try_process_job(job):
            missing = self.missing_inputs(job)
            self.pending_jobs[(job.from_key, job.to_key)] = job
            reason = "waiting for " + ", ".join(missing)
            self.publish_summary(job, "waiting_for_inputs", reason, False)

    def flush_pending_jobs(self) -> None:
        ready_keys = [key for key, job in self.pending_jobs.items() if self.has_inputs(job)]
        for key in ready_keys:
            job = self.pending_jobs.pop(key)
            self.get_logger().info(f"Pending verification job inputs are ready: {job.from_uid} -> {job.to_uid}")
            self.try_process_job(job)

    def missing_inputs(self, job: GeometricVerificationJob) -> list[str]:
        missing = []
        if not self.cloud_store.has(job.from_key):
            missing.append(f"cloud:{job.from_uid}")
        if not self.cloud_store.has(job.to_key):
            missing.append(f"cloud:{job.to_uid}")
        if not self.image_store.has(job.from_key):
            missing.append(f"image:{job.from_uid}")
        if not self.image_store.has(job.to_key):
            missing.append(f"image:{job.to_uid}")
        return missing

    def has_inputs(self, job: GeometricVerificationJob) -> bool:
        return (
            self.cloud_store.has(job.from_key)
            and self.cloud_store.has(job.to_key)
            and self.image_store.has(job.from_key)
            and self.image_store.has(job.to_key)
        )

    def try_process_job(self, job: GeometricVerificationJob) -> bool:
        job_started_at = time.monotonic()
        from_cloud = self.cloud_store.get(job.from_key)
        to_cloud = self.cloud_store.get(job.to_key)
        from_image = self.image_store.get(job.from_key)
        to_image = self.image_store.get(job.to_key)
        if from_cloud is None or to_cloud is None or from_image is None or to_image is None:
            missing = self.missing_inputs(job)
            self.get_logger().info(
                f"Verification job waiting: {job.from_uid} -> {job.to_uid}; missing={missing}"
            )
            return False

        self.get_logger().info(
            f"Processing verification job: {job.from_uid} -> {job.to_uid}; "
            f"backend={self.verifier_backend}; "
            f"cloud_points=({from_cloud.point_count},{to_cloud.point_count}); "
            f"images=({from_image.msg.width}x{from_image.msg.height},{to_image.msg.width}x{to_image.msg.height})"
        )

        try:
            from_summary = summarize_cloud(from_cloud.msg, max_points=self.max_summary_points)
            to_summary = summarize_cloud(to_cloud.msg, max_points=self.max_summary_points)
            self.validate_summary(from_summary, "from")
            self.validate_summary(to_summary, "to")
        except (PointCloudSummaryError, ValueError) as exc:
            elapsed = time.monotonic() - job_started_at
            self.get_logger().info(
                f"Finished verification job: {job.from_uid} -> {job.to_uid}; "
                f"status=rejected elapsed={elapsed:.3f}s reason={exc}"
            )
            self.publish_result(job, "rejected", str(exc), False)
            return True

        try:
            if self.verifier_backend == "mast3r_symmetric":
                estimate = self.mast3r_estimator.estimate(
                    from_image.msg,
                    to_image.msg,
                    from_cloud.msg,
                    to_cloud.msg,
                )
            else:
                estimate = estimate_geometric_constraint(
                    from_image.msg,
                    to_image.msg,
                    from_cloud.msg,
                    to_cloud.msg,
                    self.config,
                )
        except Exception as exc:
            elapsed = time.monotonic() - job_started_at
            self.get_logger().info(
                f"Finished verification job: {job.from_uid} -> {job.to_uid}; "
                f"status=rejected elapsed={elapsed:.3f}s reason={exc}"
            )
            self.publish_result(job, "rejected", str(exc), False, from_summary, to_summary)
            return True

        constraint_msg = String()
        constraint_msg.data = make_pose_constraint_json(
            job,
            estimate.sim3.data,
            estimate.match_count,
            estimate.correspondence_count,
            estimate.sim3.inlier_count,
            estimate.sim3.rmse,
            estimate.confidence,
            self.verifier_backend,
        )
        self.pose_constraint_pub.publish(constraint_msg)
        elapsed = time.monotonic() - job_started_at
        self.get_logger().info(
            f"Finished verification job: {job.from_uid} -> {job.to_uid}; "
            f"status=verified elapsed={elapsed:.3f}s matches={estimate.match_count} "
            f"correspondences={estimate.correspondence_count} inliers={estimate.sim3.inlier_count} "
            f"rmse={estimate.sim3.rmse:.4f} inlier_ratio="
            f"{estimate.sim3.inlier_count / max(1, estimate.correspondence_count):.3f}"
        )
        reason = (
            f"verified {self.verifier_backend} Sim(3): matches={estimate.match_count}, "
            f"correspondences={estimate.correspondence_count}, inliers={estimate.sim3.inlier_count}, "
            f"rmse={estimate.sim3.rmse:.4f} m"
        )
        self.publish_result(job, "verified", reason, True, from_summary, to_summary)
        return True

    def validate_summary(self, summary: dict, label: str) -> None:
        if int(summary["finite_count"]) < self.min_points:
            raise ValueError(f"{label} cloud has too few finite points: {summary['finite_count']}")
        if float(summary["radius_p90"]) <= 1e-6:
            raise ValueError(f"{label} cloud radius is degenerate")

    def publish_result(
        self,
        job: GeometricVerificationJob,
        status: str,
        reason: str,
        constraint_ready: bool,
        from_summary: dict | None = None,
        to_summary: dict | None = None,
    ) -> None:
        result_msg = String()
        result_msg.data = make_result_json(job, status, reason, constraint_ready, from_summary, to_summary)
        self.result_pub.publish(result_msg)
        self.publish_summary(job, status, reason, constraint_ready)

    def publish_summary(self, job: GeometricVerificationJob, status: str, reason: str, constraint_ready: bool) -> None:
        similarity = float(job.candidate.get("similarity", 0.0))
        ready_text = "true" if constraint_ready else "false"
        msg = String()
        msg.data = (
            f"{job.from_uid} -> {job.to_uid} similarity={similarity:.4f} "
            f"status={status} constraint_ready={ready_text} reason={reason}"
        )
        self.summary_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotGeometricVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
