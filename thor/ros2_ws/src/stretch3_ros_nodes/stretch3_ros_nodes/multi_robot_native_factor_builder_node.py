#!/usr/bin/env python3
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_native_cache import NativeKeyframeManifest
from stretch3_ros_nodes.native_frame_cache import load_frame_record
from stretch3_ros_nodes.native_imports import configure_mast3r_imports, default_cache_root
from stretch3_ros_nodes.native_messages import dumps, loads, save_edge_cache, tensor_shape


class MultiRobotNativeFactorBuilderNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_factor_builder_node")
        self.declare_parameter("candidate_topic", "/multi_robot/native_retrieval_candidates")
        self.declare_parameter("edge_topic", "/multi_robot/native_factor_edges")
        self.declare_parameter("summary_topic", "/multi_robot/native_factor_summaries")
        self.declare_parameter("edge_cache_dir", default_cache_root() + "/edges")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("mast3r_config_path", "config/base.yaml")
        self.declare_parameter("mast3r_model_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("q_conf", 1.5)
        self.declare_parameter("min_match_frac", 0.1)

        self.candidate_topic = self.get_parameter("candidate_topic").value
        self.edge_cache_dir = self.get_parameter("edge_cache_dir").value
        self.mast3r_slam_root = self.get_parameter("mast3r_slam_root").value
        self.mast3r_config_path = self.get_parameter("mast3r_config_path").value
        self.mast3r_model_path = self.get_parameter("mast3r_model_path").value.strip() or None
        self.device = self.get_parameter("device").value
        self.q_conf = float(self.get_parameter("q_conf").value)
        self.min_match_frac = float(self.get_parameter("min_match_frac").value)
        self._loaded = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.edge_pub = self.create_publisher(String, self.get_parameter("edge_topic").value, qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("summary_topic").value, qos)
        self.sub = self.create_subscription(String, self.candidate_topic, self.on_candidate, qos)
        self.get_logger().info(
            f"Native factor builder listening on {self.candidate_topic}; q_conf={self.q_conf}, min_match_frac={self.min_match_frac}"
        )

    def _ensure_loaded(self):
        if self._loaded:
            return
        started = time.monotonic()
        root = configure_mast3r_imports(self.mast3r_slam_root)
        from mast3r_slam.config import load_config
        from mast3r_slam.mast3r_utils import load_mast3r, mast3r_match_symmetric
        load_config(str(root / self.mast3r_config_path))
        model_path = self.mast3r_model_path or str(root / "checkpoints" / "MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth")
        self.model = load_mast3r(model_path, device=self.device)
        self.match_symmetric = mast3r_match_symmetric
        self.torch = __import__("torch")
        self.get_logger().info(f"Loaded MASt3R factor builder model in {time.monotonic() - started:.2f}s")
        self._loaded = True

    def _record_from_candidate(self, data, prefix: str):
        robot = data[f"{prefix}_robot"]
        kf_id = int(data[f"{prefix}_kf_id"])
        uid = data[f"{prefix}_uid"]
        path = data[f"{prefix}_cache_path"]
        global_idx = int(data[f"{prefix}_global_idx"])
        manifest = NativeKeyframeManifest(robot, kf_id, uid, path, "mast3r_native_keyframe_cache_v1", {})
        return load_frame_record(manifest, global_idx, self.device)

    def on_candidate(self, msg: String) -> None:
        self._ensure_loaded()
        started = time.monotonic()
        data = loads(msg.data)
        try:
            a = self._record_from_candidate(data, "query")
            b = self._record_from_candidate(data, "match")
            with self.torch.inference_mode():
                idx_i2j, idx_j2i, valid_match_j, valid_match_i, qii, qjj, qji, qij = self.match_symmetric(
                    self.model,
                    a.payload["feat"], a.payload["pos"],
                    b.payload["feat"], b.payload["pos"],
                    [a.payload["img_true_shape"]], [b.payload["img_true_shape"]],
                )
                batch = self.torch.arange(idx_i2j.shape[0], device=idx_i2j.device)[:, None].repeat(1, idx_i2j.shape[1])
                qj = self.torch.sqrt(qii[batch, idx_i2j] * qji)
                qi = self.torch.sqrt(qjj[batch, idx_j2i] * qij)
                valid_j = valid_match_j & (qj > self.q_conf)
                valid_i = valid_match_i & (qi > self.q_conf)
                match_frac_j = float((valid_j.sum() / (valid_j.shape[1] * valid_j.shape[2])).detach().cpu())
                match_frac_i = float((valid_i.sum() / (valid_i.shape[1] * valid_i.shape[2])).detach().cpu())
            accepted = min(match_frac_i, match_frac_j) >= self.min_match_frac
            reason = "accepted" if accepted else "low_match_frac"
            edge_uid = f"edge_{a.keyframe_uid}__{b.keyframe_uid}"
            metadata = {
                "from_robot": a.robot_id, "from_kf_id": a.kf_id, "from_uid": a.keyframe_uid, "from_global_idx": a.global_index, "from_cache_path": a.cache_path,
                "to_robot": b.robot_id, "to_kf_id": b.kf_id, "to_uid": b.keyframe_uid, "to_global_idx": b.global_index, "to_cache_path": b.cache_path,
                "match_frac_i": match_frac_i, "match_frac_j": match_frac_j, "q_conf": self.q_conf, "min_match_frac": self.min_match_frac,
                "accepted": accepted, "rejected_reason": None if accepted else reason, "edge_type": "cross_robot_loop",
            }
            edge_path = save_edge_cache(self.edge_cache_dir, edge_uid, {
                "idx_i2j": idx_i2j.detach().cpu(), "idx_j2i": idx_j2i.detach().cpu(),
                "valid_match_j": valid_match_j.detach().cpu(), "valid_match_i": valid_match_i.detach().cpu(),
                "Q_ii2jj": qj.detach().cpu(), "Q_jj2ii": qi.detach().cpu(),
            }, metadata)
            payload = dict(metadata)
            payload.update({
                "schema": "multi_robot_native_dense_factor_edge_v1", "edge_uid": edge_uid, "edge_cache_path": edge_path,
                "tensor_shapes": {"idx_i2j": tensor_shape(idx_i2j), "idx_j2i": tensor_shape(idx_j2i), "valid_match_j": tensor_shape(valid_match_j), "valid_match_i": tensor_shape(valid_match_i), "Q_ii2jj": tensor_shape(qj), "Q_jj2ii": tensor_shape(qi)},
                "elapsed_sec": time.monotonic() - started,
            })
            out = String(); out.data = dumps(payload); self.edge_pub.publish(out)
            summary = String(); summary.data = dumps({"schema":"multi_robot_native_factor_summary_v1", **payload}); self.summary_pub.publish(summary)
            self.get_logger().info(
                f"Native factor {edge_uid}: accepted={accepted} reason={reason} match_frac_i={match_frac_i:.4f} "
                f"match_frac_j={match_frac_j:.4f} q_conf={self.q_conf} elapsed={time.monotonic()-started:.2f}s"
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed native factor candidate: {exc}")


def main(args=None):
    rclpy.init(args=args); node = MultiRobotNativeFactorBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
