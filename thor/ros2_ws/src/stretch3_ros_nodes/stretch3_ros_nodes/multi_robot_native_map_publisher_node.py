#!/usr/bin/env python3
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from stretch3_ros_nodes.cross_robot_native_cache import NativeKeyframeManifest
from stretch3_ros_nodes.native_frame_cache import load_frame_record, record_to_frame
from stretch3_ros_nodes.native_imports import configure_mast3r_imports
from stretch3_ros_nodes.native_messages import dumps
from stretch3_ros_nodes.optimized_cloud_transformer import make_cloud_from_point_array, make_point_array
from stretch3_ros_nodes.pointcloud_voxel_filter import voxel_downsample_points


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
        self.declare_parameter("chunk_max_points", 250000)
        self.declare_parameter("chunk_qos_depth", 8)
        self.declare_parameter("chunk_publish_delay_sec", 0.01)
        self.declare_parameter("voxel_leaf_size", 0.0)
        self.declare_parameter("publish_period_sec", 1.0)

        configure_mast3r_imports(self.get_parameter("mast3r_slam_root").value)
        self.device = self.get_parameter("device").value
        self.output_frame = self.get_parameter("output_frame").value
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.max_merged_points = int(self.get_parameter("max_merged_points").value)
        self.chunk_max_points = max(1, int(self.get_parameter("chunk_max_points").value))
        self.chunk_qos_depth = max(1, int(self.get_parameter("chunk_qos_depth").value))
        self.chunk_publish_delay_sec = max(0.0, float(self.get_parameter("chunk_publish_delay_sec").value))
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.records = {}
        self.poses = {}
        self.revision = 0
        self.dirty = False

        control_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=50)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=self.chunk_qos_depth)
        self.map_pub = self.create_publisher(PointCloud2, self.get_parameter("optimized_map_topic").value, map_qos)
        self.summary_pub = self.create_publisher(String, self.get_parameter("optimized_map_summary_topic").value, control_qos)
        self.sub = self.create_subscription(String, self.get_parameter("optimized_pose_topic").value, self.on_pose, control_qos)
        period = float(self.get_parameter("publish_period_sec").value)
        self.timer = self.create_timer(period, self.publish_map) if period > 0 else None
        self.get_logger().info(
            f"Native map publisher listening on {self.get_parameter('optimized_pose_topic').value}; output={self.get_parameter('optimized_map_topic').value} "
            f"min_confidence={self.min_confidence} max_merged_points={self.max_merged_points} "
            f"chunk_max_points={self.chunk_max_points} voxel={self.voxel_leaf_size} "
            f"map_qos_depth={self.chunk_qos_depth} chunk_delay={self.chunk_publish_delay_sec}"
        )

    def on_pose(self, msg: String):
        try:
            data = json.loads(msg.data)
            key = (data["robot_id"], int(data["kf_id"]))
            self.poses[key] = data
            if key not in self.records:
                manifest = NativeKeyframeManifest(data["robot_id"], int(data["kf_id"]), data["keyframe_uid"], data["cache_path"], "mast3r_native_keyframe_cache_v1", {})
                self.records[key] = load_frame_record(
                    manifest,
                    int(data.get("global_index", len(self.records))),
                    self.device,
                    tensor_keys=("sim3_data", "X_canon", "C"),
                )
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
        arr = make_point_array(xyz, rgb, conf_values)
        # Native 16 streams chunks to keep memory bounded. A global voxel filter would
        # require holding a full voxel table for the entire map; apply per-keyframe
        # voxel filtering when requested, and keep the default at 0.0 for exact output.
        return voxel_downsample_points(arr, self.voxel_leaf_size)

    def _native_sim3(self, data):
        import lietorch, torch
        return lietorch.Sim3(torch.as_tensor(data, dtype=torch.float32, device=self.device).reshape(1, -1))

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

    def _publish_chunk(self, chunk, revision: int, chunk_index: int, chunk_count: int):
        if chunk_count > 1:
            frame_id = f"kf_999999_{revision}_{chunk_index}_{chunk_count}"
        else:
            frame_id = f"native_map_rev_{revision}"
        msg = make_cloud_from_point_array(chunk, self.get_clock().now().to_msg(), frame_id, frame_id).msg
        self.map_pub.publish(msg)
        if self.chunk_publish_delay_sec > 0.0 and chunk_count > 1:
            time.sleep(self.chunk_publish_delay_sec)
        return int(msg.width) * int(msg.height)

    def publish_map(self):
        if not self.dirty or not self.records:
            return
        started = time.monotonic()
        keys = sorted(self.records)

        # First pass: count points only. Arrays are immediately discarded so we can
        # compute chunk_count without holding the full map in memory.
        total_points = 0
        ready_keys = []
        for key in keys:
            arr = self._cloud_for_key(key)
            if arr is None or arr.shape[0] == 0:
                continue
            ready_keys.append(key)
            total_points += int(arr.shape[0])
            del arr
        if total_points <= 0:
            return

        capped_points = total_points if self.max_merged_points <= 0 else min(total_points, self.max_merged_points)
        stride = 1 if total_points <= capped_points else max(1, total_points // capped_points)
        chunk_count = max(1, math.ceil(capped_points / self.chunk_max_points))
        self.revision += 1

        chunk_parts = []
        chunk_points = 0
        published_points = 0
        published_chunks = 0
        global_offset = 0

        # Second pass: regenerate one keyframe at a time, append only to the current
        # chunk, publish the chunk, then release it. No full-map concatenate occurs.
        for key in ready_keys:
            arr = self._cloud_for_key(key)
            if arr is None or arr.shape[0] == 0:
                continue
            source_len = int(arr.shape[0])
            if stride > 1:
                local_indices = np.arange(source_len, dtype=np.int64) + global_offset
                arr = arr[(local_indices % stride) == 0]
            global_offset += source_len
            if arr.shape[0] == 0:
                continue

            start_idx = 0
            while start_idx < arr.shape[0] and published_points < capped_points:
                remaining_total = capped_points - published_points
                remaining_chunk = self.chunk_max_points - chunk_points
                take = min(int(arr.shape[0] - start_idx), remaining_total, remaining_chunk)
                if take <= 0:
                    break
                chunk_parts.append(arr[start_idx:start_idx + take])
                chunk_points += take
                published_points += take
                start_idx += take

                if chunk_points >= self.chunk_max_points or published_points >= capped_points:
                    chunk = chunk_parts[0] if len(chunk_parts) == 1 else np.concatenate(chunk_parts)
                    published_chunks += 1
                    self._publish_chunk(chunk, self.revision, published_chunks, chunk_count)
                    del chunk
                    chunk_parts = []
                    chunk_points = 0
            del arr

        if chunk_parts:
            chunk = chunk_parts[0] if len(chunk_parts) == 1 else np.concatenate(chunk_parts)
            published_chunks += 1
            self._publish_chunk(chunk, self.revision, published_chunks, chunk_count)
            del chunk

        summary = {
            "schema":"multi_robot_optimized_map_summary_v1", "native_schema":"multi_robot_native_map_summary_v1",
            "revision":self.revision, "keyframes":len(self.records), "point_count":int(capped_points), "frame_id":self.output_frame,
            "min_confidence":self.min_confidence, "max_merged_points":self.max_merged_points, "voxel_leaf_size":self.voxel_leaf_size,
            "chunk_count":int(chunk_count), "published_chunks":int(published_chunks), "chunk_max_points":self.chunk_max_points,
            "chunk_qos_depth":self.chunk_qos_depth, "chunk_publish_delay_sec":self.chunk_publish_delay_sec,
            "streaming_chunks":True, "raw_point_count":int(total_points), "elapsed_sec":time.monotonic()-started,
        }
        out=String(); out.data=dumps(summary); self.summary_pub.publish(out)
        self.get_logger().info(
            f"Published native optimized map revision={self.revision} keyframes={len(self.records)} point_count={int(capped_points)} "
            f"raw_point_count={int(total_points)} min_confidence={self.min_confidence} max_merged_points={self.max_merged_points} "
            f"chunk_count={chunk_count} published_chunks={published_chunks} chunk_max_points={self.chunk_max_points} "
            f"qos_depth={self.chunk_qos_depth} chunk_delay={self.chunk_publish_delay_sec} streaming=True"
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
