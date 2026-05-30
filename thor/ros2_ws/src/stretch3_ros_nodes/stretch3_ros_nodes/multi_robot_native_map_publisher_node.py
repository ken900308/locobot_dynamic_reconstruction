#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from stretch3_ros_nodes.cross_robot_native_cache import NativeKeyframeManifest
from stretch3_ros_nodes.native_frame_cache import load_frame_record, record_to_frame
from stretch3_ros_nodes.native_imports import configure_mast3r_imports
from stretch3_ros_nodes.native_messages import dumps
from stretch3_ros_nodes.optimized_cloud_transformer import make_cloud_from_point_array, make_point_array
from stretch3_ros_nodes.pointcloud_voxel_filter import cap_points, voxel_downsample_points
from stretch3_ros_nodes.sim3_math import Sim3


class MultiRobotNativeMapPublisherNode(Node):
    def __init__(self):
        super().__init__("multi_robot_native_map_publisher_node")
        self.declare_parameter("optimized_pose_topic", "/multi_robot/native_optimized_keyframe_poses")
        self.declare_parameter("optimized_map_topic", "/multi_robot/optimized_map_points")
        self.declare_parameter("optimized_map_summary_topic", "/multi_robot/optimized_map_summaries")
        self.declare_parameter("output_frame", "multi_robot_optimized_map")
        self.declare_parameter("mast3r_slam_root", "/workspace/thor/MASt3R-SLAM")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("min_confidence", 0.95)
        self.declare_parameter("max_merged_points", 5000000)
        self.declare_parameter("voxel_leaf_size", 0.0)
        self.declare_parameter("publish_period_sec", 1.0)

        configure_mast3r_imports(self.get_parameter("mast3r_slam_root").value)
        self.device = self.get_parameter("device").value
        self.output_frame = self.get_parameter("output_frame").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.max_merged_points = int(self.get_parameter("max_merged_points").value)
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.records = {}
        self.poses = {}
        self.revision = 0
        self.dirty = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.map_pub = self.create_publisher(PointCloud2, self.get_parameter("optimized_map_topic").value, qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("optimized_map_summary_topic").value, qos)
        self.sub = self.create_subscription(String, self.get_parameter("optimized_pose_topic").value, self.on_pose, qos)
        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self.publish_map) if period > 0 else None
        self.get_logger().info(
            f"Native map publisher listening on {self.get_parameter('optimized_pose_topic').value}; output={self.get_parameter('optimized_map_topic').value} "
            f"min_confidence={self.min_confidence} max_merged_points={self.max_merged_points} voxel={self.voxel_leaf_size}"
        )

    def on_pose(self, msg: String):
        try:
            data = json.loads(msg.data)
            key = (data["robot_id"], int(data["kf_id"]))
            self.poses[key] = data
            if key not in self.records:
                manifest = NativeKeyframeManifest(data["robot_id"], int(data["kf_id"]), data["keyframe_uid"], data["cache_path"], "mast3r_native_keyframe_cache_v1", {})
                self.records[key] = load_frame_record(manifest, int(data.get("global_index", len(self.records))), self.device)
            self.dirty = True
        except Exception as exc:
            self.get_logger().warn(f"Rejected native optimized pose: {exc}")

    def _cloud_for_key(self, key):
        data = self.poses[key]
        record = self.records[key]
        frame = record_to_frame(record)
        frame.T_WC = self._native_sim3(data["optimized_sim3_data"])
        x_world = frame.T_WC.act(frame.X_canon).detach().cpu().numpy().reshape(-1, 3)
        conf = frame.get_average_conf().detach().cpu().numpy().reshape(-1)
        valid = np.isfinite(x_world).all(axis=1) & np.isfinite(conf) & (conf >= self.min_confidence)
        if not np.any(valid):
            return None
        xyz = x_world[valid].astype(np.float32)
        conf_values = conf[valid].astype(np.float32)
        colors = self._colors(frame, valid)
        rgb = (colors[:,0].astype(np.uint32) << 16) | (colors[:,1].astype(np.uint32) << 8) | colors[:,2].astype(np.uint32)
        return make_point_array(xyz, rgb, conf_values)

    def _native_sim3(self, data):
        import lietorch, torch
        return lietorch.Sim3(torch.as_tensor(data, dtype=torch.float32, device=self.device).reshape(1, -1))

    def _colors(self, frame, valid_mask):
        uimg = frame.uimg.detach().cpu().numpy()
        if uimg.ndim == 4:
            uimg = uimg[0]
        colors = uimg.reshape(-1, uimg.shape[-1])[:, :3]
        colors = colors[valid_mask]
        if colors.dtype != np.uint8:
            if float(np.nanmax(colors)) <= 1.5:
                colors = colors * 255.0
            colors = np.clip(colors, 0, 255).astype(np.uint8)
        return colors

    def publish_map(self):
        if not self.dirty or not self.records:
            return
        started = time.monotonic()
        arrays = []
        for key in sorted(self.records):
            arr = self._cloud_for_key(key)
            if arr is not None:
                arrays.append(arr)
        if not arrays:
            return
        merged = np.concatenate(arrays)
        merged = voxel_downsample_points(merged, self.voxel_leaf_size)
        merged = cap_points(merged, self.max_merged_points)
        self.revision += 1
        msg = make_cloud_from_point_array(merged, self.get_clock().now().to_msg(), self.output_frame, f"native_map_rev_{self.revision}").msg
        self.map_pub.publish(msg)
        point_count = int(msg.width) * int(msg.height)
        chunk_count = 1
        summary = {
            "schema":"multi_robot_optimized_map_summary_v1", "native_schema":"multi_robot_native_map_summary_v1",
            "revision":self.revision, "keyframes":len(self.records), "point_count":point_count, "frame_id":self.output_frame,
            "min_confidence":self.min_confidence, "max_merged_points":self.max_merged_points, "voxel_leaf_size":self.voxel_leaf_size,
            "chunk_count":chunk_count, "elapsed_sec":time.monotonic()-started,
        }
        out=String(); out.data=dumps(summary); self.summary_pub.publish(out)
        self.get_logger().info(
            f"Published native optimized map revision={self.revision} keyframes={len(self.records)} point_count={point_count} "
            f"min_confidence={self.min_confidence} max_merged_points={self.max_merged_points} chunk_count={chunk_count}"
        )
        self.dirty = False


def main(args=None):
    rclpy.init(args=args); node=MultiRobotNativeMapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
