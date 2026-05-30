#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_native_cache import NativeKeyframeManifest
from stretch3_ros_nodes.native_frame_cache import NativeFrameCollection, load_frame_record
from stretch3_ros_nodes.native_imports import configure_mast3r_imports
from stretch3_ros_nodes.native_messages import dumps, load_edge_cache
from stretch3_ros_nodes.sim3_math import Sim3


def _sim3_to_list(native_sim3) -> list[float]:
    return [float(x) for x in native_sim3.data.detach().cpu().reshape(-1).tolist()]


class MultiRobotNativeOptimizerNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_optimizer_node")
        self.declare_parameter("edge_topic", "/multi_robot/native_factor_edges")
        self.declare_parameter("optimized_pose_topic", "/multi_robot/native_optimized_keyframe_poses")
        self.declare_parameter("compat_optimized_pose_topic", "/multi_robot/optimized_keyframe_poses")
        self.declare_parameter("summary_topic", "/multi_robot/native_optimizer_summaries")
        self.declare_parameter("anchor_robot", "robot1")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("mast3r_config_path", "config/base.yaml")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("use_calib", False)

        self.anchor_robot = self.get_parameter("anchor_robot").value
        self.device = self.get_parameter("device").value
        self.mast3r_slam_root = self.get_parameter("mast3r_slam_root").value
        self.mast3r_config_path = self.get_parameter("mast3r_config_path").value
        self.use_calib = bool(self.get_parameter("use_calib").value)
        self._loaded = False
        self.edge_keys = set()
        self.edge_count = 0

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.pose_pub = self.create_publisher(String, self.get_parameter("optimized_pose_topic").value, qos)
        self.compat_pose_pub = self.create_publisher(String, self.get_parameter("compat_optimized_pose_topic").value, qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("summary_topic").value, qos)
        self.sub = self.create_subscription(String, self.get_parameter("edge_topic").value, self.on_edge, qos)
        self.get_logger().info(f"Native optimizer listening on {self.get_parameter('edge_topic').value}; anchor_robot={self.anchor_robot}")

    def _ensure_loaded(self):
        if self._loaded:
            return
        root = configure_mast3r_imports(self.mast3r_slam_root)
        from mast3r_slam.config import load_config, config
        from mast3r_slam.global_opt import FactorGraph
        load_config(str(root / self.mast3r_config_path))
        self.config = config
        self.torch = __import__("torch")
        self.lietorch = __import__("lietorch")
        self.frames = NativeFrameCollection(self.device)
        self.graph = FactorGraph(None, self.frames, None, self.device)
        self._loaded = True

    def _ensure_record(self, metadata, prefix: str):
        idx = int(metadata[f"{prefix}_global_idx"])
        if self.frames.has(idx):
            return
        manifest = NativeKeyframeManifest(metadata[f"{prefix}_robot"], int(metadata[f"{prefix}_kf_id"]), metadata[f"{prefix}_uid"], metadata[f"{prefix}_cache_path"], "mast3r_native_keyframe_cache_v1", {})
        tensor_keys = ("sim3_data", "X_canon", "C")
        if self.use_calib:
            tensor_keys = ("sim3_data", "img", "X_canon", "C", "K")
        self.frames.upsert(load_frame_record(manifest, idx, self.device, tensor_keys=tensor_keys))

    def on_edge(self, msg: String) -> None:
        self._ensure_loaded()
        data = json.loads(msg.data)
        if not data.get("accepted", False):
            return
        edge_uid = data["edge_uid"]
        if edge_uid in self.edge_keys:
            return
        started = time.monotonic()
        try:
            edge = load_edge_cache(data["edge_cache_path"])
            meta = edge["metadata"]
            self._ensure_record(meta, "from")
            self._ensure_record(meta, "to")
            tensors = edge["tensors"]
            ii = self.torch.as_tensor([int(meta["from_global_idx"])], dtype=self.torch.long, device=self.device)
            jj = self.torch.as_tensor([int(meta["to_global_idx"])], dtype=self.torch.long, device=self.device)
            edge_tensors = {
                "ii": ii,
                "jj": jj,
                "idx_ii2jj": tensors["idx_i2j"].to(self.device),
                "idx_jj2ii": tensors["idx_j2i"].to(self.device),
                "valid_match_j": tensors["valid_match_j"].to(self.device),
                "valid_match_i": tensors["valid_match_i"].to(self.device),
                "Q_ii2jj": tensors["Q_ii2jj"].to(self.device),
                "Q_jj2ii": tensors["Q_jj2ii"].to(self.device),
            }
            for name, value in edge_tensors.items():
                current = getattr(self.graph, name)
                setattr(self.graph, name, value if current.numel() == 0 else self.torch.cat([current, value], dim=0))
            self.edge_keys.add(edge_uid); self.edge_count += 1
            solver = "gauss_newton_calib" if self.use_calib and self.frames.K is not None else "gauss_newton_rays"
            if solver == "gauss_newton_calib":
                self.graph.K = self.frames.K
                self.graph.solve_GN_calib()
            else:
                self.graph.solve_GN_rays()
            updated = self.publish_poses()
            unique = int(self.graph.get_unique_kf_idx().numel()) if self.edge_count > 0 else 0
            summary = {
                "schema":"multi_robot_native_optimizer_summary_v1", "reason":edge_uid, "anchor_robot":self.anchor_robot,
                "keyframes":len(self.frames), "edges":self.edge_count, "unique_optimized":unique,
                "pin":int(self.config["local_opt"]["pin"]), "solver":solver, "iterations":int(self.config["local_opt"]["max_iters"]),
                "updated_poses":updated, "elapsed_sec":time.monotonic()-started,
            }
            out=String(); out.data=dumps(summary); self.summary_pub.publish(out)
            self.get_logger().info(
                f"Native optimizer update after {edge_uid}: keyframes={len(self.frames)} edges={self.edge_count} "
                f"unique_optimized={unique} pin={summary['pin']} solver={solver} iterations={summary['iterations']} updated_poses={updated}"
            )
        except Exception as exc:
            self.get_logger().warn(f"Native optimization failed for {edge_uid}: {exc}")

    def publish_poses(self) -> int:
        count = 0
        for rec in self.frames.records():
            frame = self.frames[rec.global_index]
            sim3_data = _sim3_to_list(frame.T_WC)
            payload = {
                "schema":"multi_robot_optimized_keyframe_pose_v1",
                "native_schema":"multi_robot_native_optimized_keyframe_pose_v1",
                "robot_id":rec.robot_id, "kf_id":rec.kf_id, "keyframe_uid":rec.keyframe_uid,
                "global_index":rec.global_index, "cache_path":rec.cache_path, "anchor_robot":self.anchor_robot,
                "optimized_sim3_data":sim3_data, "sim3_layout":"tx_ty_tz_qx_qy_qz_qw_scale",
                "native_edge_count":self.edge_count,
            }
            msg=String(); msg.data=dumps(payload)
            self.pose_pub.publish(msg); self.compat_pose_pub.publish(msg)
            count += 1
        return count


def main(args=None):
    rclpy.init(args=args); node=MultiRobotNativeOptimizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
