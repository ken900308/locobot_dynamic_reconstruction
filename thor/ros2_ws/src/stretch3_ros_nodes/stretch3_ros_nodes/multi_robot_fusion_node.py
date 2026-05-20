#!/usr/bin/env python3
"""
Milestone 1A multi-robot pointcloud fusion node.

This node keeps the first milestone intentionally simple:
- subscribe to namespaced MASt3R keyframe pointclouds
- apply a manually configured per-robot transform into a shared global frame
- republish transformed clouds for Unity/global visualization

It does not perform visual matching, AprilTag estimation, or PGO.
"""

import math
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


def _parse_csv(value: str) -> List[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _parse_float_list(value: str, expected_len: int, default: List[float]) -> List[float]:
    try:
        parts = [float(item.strip()) for item in value.split(",") if item.strip()]
    except Exception:
        return list(default)
    return parts if len(parts) == expected_len else list(default)


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float32,
    )


class MultiRobotFusionNode(Node):
    def __init__(self):
        super().__init__("multi_robot_fusion_node")

        self.declare_parameter("global_frame", "global_map")
        self.declare_parameter("output_topic", "/multi_robot/global_pointcloud")
        self.declare_parameter("robot_ids", "robot1,robot2")
        self.declare_parameter("qos_reliable", True)

        self.global_frame = self.get_parameter("global_frame").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        robot_ids_text = self.get_parameter("robot_ids").get_parameter_value().string_value
        self.robot_ids = _parse_csv(robot_ids_text) or ["robot1", "robot2"]
        qos_reliable = self.get_parameter("qos_reliable").get_parameter_value().bool_value

        reliability = ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT
        qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.transforms: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self.counts: Dict[str, int] = {}
        self._subscriptions = []

        for robot_id in self.robot_ids:
            default_topic = f"/{robot_id}/mast3r/frame_pointcloud"
            self.declare_parameter(f"{robot_id}.input_topic", default_topic)
            self.declare_parameter(f"{robot_id}.translation", "0.0,0.0,0.0")
            self.declare_parameter(f"{robot_id}.rotation_xyzw", "0.0,0.0,0.0,1.0")

            topic = self.get_parameter(f"{robot_id}.input_topic").get_parameter_value().string_value
            translation = _parse_float_list(
                self.get_parameter(f"{robot_id}.translation").get_parameter_value().string_value,
                3,
                [0.0, 0.0, 0.0],
            )
            rotation = _parse_float_list(
                self.get_parameter(f"{robot_id}.rotation_xyzw").get_parameter_value().string_value,
                4,
                [0.0, 0.0, 0.0, 1.0],
            )

            R = _quat_to_rot(rotation[0], rotation[1], rotation[2], rotation[3])
            t = np.asarray(translation, dtype=np.float32)
            self.transforms[robot_id] = (R, t)
            self.counts[robot_id] = 0

            self._subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, rid=robot_id: self.on_cloud(rid, msg),
                    qos,
                )
            )

            self.get_logger().info(
                f"Robot {robot_id}: input={topic}, translation={translation}, rotation_xyzw={rotation}"
            )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.get_logger().info(f"Publishing fused global pointclouds on {self.output_topic}")
        self.get_logger().info(f"Output frame convention: points are in {self.global_frame}; frame_id keeps robot/kf identity for Unity")

    def on_cloud(self, robot_id: str, msg: PointCloud2):
        try:
            out = self.transform_cloud(robot_id, msg)
            self.pub.publish(out)
            self.counts[robot_id] += 1
            if self.counts[robot_id] % 10 == 1:
                self.get_logger().info(
                    f"Published fused cloud from {robot_id}: {msg.width} points, count={self.counts[robot_id]}"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to fuse cloud from {robot_id}: {exc}")

    def transform_cloud(self, robot_id: str, msg: PointCloud2) -> PointCloud2:
        if msg.point_step < 12:
            raise ValueError(f"unsupported point_step={msg.point_step}")

        field_offsets = {field.name: field.offset for field in msg.fields}
        for required in ("x", "y", "z"):
            if required not in field_offsets:
                raise ValueError(f"missing field: {required}")

        data = bytearray(msg.data)
        point_count = int(msg.width) * int(msg.height)
        if point_count <= 0:
            return self.make_output_msg(robot_id, msg, bytes(data))

        dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": ["<f4", "<f4", "<f4"],
                "offsets": [field_offsets["x"], field_offsets["y"], field_offsets["z"]],
                "itemsize": msg.point_step,
            }
        )
        points = np.ndarray(shape=(point_count,), dtype=dtype, buffer=data)
        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1).astype(np.float32, copy=False)

        R, t = self.transforms[robot_id]
        xyz_out = (xyz @ R.T) + t

        points["x"] = xyz_out[:, 0]
        points["y"] = xyz_out[:, 1]
        points["z"] = xyz_out[:, 2]

        return self.make_output_msg(robot_id, msg, bytes(data))

    def make_output_msg(self, robot_id: str, msg: PointCloud2, data: bytes) -> PointCloud2:
        out = PointCloud2()
        out.header.stamp = msg.header.stamp

        source_id = msg.header.frame_id or "kf_unknown"
        out.header.frame_id = f"{robot_id}_{source_id}"

        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = data
        return out


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
