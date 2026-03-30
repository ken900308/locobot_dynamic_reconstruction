#!/usr/bin/env python3
"""
ROS2 IPC PointCloud Receiver
- Receives pointcloud over UDS IPC socket
- Publishes: /mast3r/frame_pointcloud (PointCloud2)
"""

import os
import socket
import struct
import threading
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose


class IPCPointCloudReceiver(Node):
    MAGIC = b"MSR1"
    VERSION = 1

    # Header: magic(4) version(1) msg_type(1) flags(2) width(4) height(4) encoding(4) timestamp_ns(8) payload_len(4)
    HEADER_STRUCT = struct.Struct("<4sBBHIIIQI")

    MSG_POINTCLOUD = 2
    MSG_POSEARRAY = 3
    ENC_XYZRGB_FLOAT32 = 10
    ENC_JSON = 99

    def __init__(self):
        super().__init__('ipc_pointcloud_receiver')

        self.declare_parameter('socket_path', '/tmp/ipc_socket/locobot/mast3r_pointcloud.sock')
        self.declare_parameter('output_topic', '/mast3r/frame_pointcloud')
        self.declare_parameter('pose_topic', '/mast3r/keyframe_pose_updates')
        self.declare_parameter('frame_id', 'mast3r_map')
        self.declare_parameter('qos_reliable', True)

        self.socket_path = self.get_parameter('socket_path').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        qos_reliable = self.get_parameter('qos_reliable').get_parameter_value().bool_value

        reliability = ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT
        pc_qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pc_publisher = self.create_publisher(PointCloud2, self.output_topic, pc_qos)
        self.pose_publisher = self.create_publisher(PoseArray, self.pose_topic, pose_qos)

        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._stop = threading.Event()
        self._count = 0
        self._last_log_time = 0.0

        self.get_logger().info(f"📡 IPC socket: {self.socket_path}")
        self.get_logger().info(f"☁️ Output PC topic: {self.output_topic}")
        self.get_logger().info(f"🔄 Output Pose topic: {self.pose_topic}")
        self.get_logger().info(f"🧭 Frame ID: {self.frame_id}")

        self._thread.start()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()

    def _serve(self):
        if os.path.exists(self.socket_path):
            try:
                os.remove(self.socket_path)
            except Exception:
                pass

        sock_dir = os.path.dirname(self.socket_path)
        if sock_dir:
            os.makedirs(sock_dir, exist_ok=True)

        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(self.socket_path)
            server.listen(1)

            while not self._stop.is_set():
                try:
                    conn, _ = server.accept()
                except Exception:
                    continue

                with conn:
                    while not self._stop.is_set():
                        header = self._recv_exact(conn, self.HEADER_STRUCT.size)
                        if not header:
                            break

                        try:
                            (magic, version, msg_type, _flags, width, height, encoding, ts_ns, payload_len) = \
                                self.HEADER_STRUCT.unpack(header)
                        except Exception:
                            break

                        if magic != self.MAGIC or version != self.VERSION:
                            break

                        payload = self._recv_exact(conn, payload_len)
                        if not payload:
                            break

                        if msg_type == self.MSG_POINTCLOUD:
                            # We repurposed the 'height' field in the IPC Header to carry the 'kf_id'
                            kf_id = height 
                            self._handle_pointcloud(width, kf_id, encoding, ts_ns, payload)
                        elif msg_type == self.MSG_POSEARRAY:
                            self._handle_posearray(ts_ns, payload)

    def _recv_exact(self, conn, size):
        data = b""
        while len(data) < size and not self._stop.is_set():
            chunk = conn.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _handle_posearray(self, ts_ns, payload):
        try:
            data = json.loads(payload.decode('utf-8'))
            msg = PoseArray()
            
            if ts_ns and ts_ns > 0:
                msg.header.stamp = Time(nanoseconds=int(ts_ns)).to_msg()
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
                
            msg.header.frame_id = data.get("frame_id", "")
            
            for pose_dict in data.get("poses", []):
                p = Pose()
                pos = pose_dict.get("position", {})
                p.position.x = float(pos.get("x", 0.0))
                p.position.y = float(pos.get("y", 0.0))
                p.position.z = float(pos.get("z", 0.0))
                
                ori = pose_dict.get("orientation", {})
                p.orientation.x = float(ori.get("x", 0.0))
                p.orientation.y = float(ori.get("y", 0.0))
                p.orientation.z = float(ori.get("z", 0.0))
                p.orientation.w = float(ori.get("w", 1.0))
                
                msg.poses.append(p)
                
            self.pose_publisher.publish(msg)
            self.get_logger().info(f"🔄 IPC PoseArray received: {len(msg.poses)} poses")
        except Exception as e:
            self.get_logger().error(f"IPC posearray handle error: {e}")

    def _handle_pointcloud(self, width, kf_id, encoding, ts_ns, payload):
        try:
            if encoding != self.ENC_XYZRGB_FLOAT32:
                self.get_logger().warn(f"Unsupported pointcloud encoding: {encoding}")
                return

            point_count = int(width) if width > 0 else 0
            if point_count <= 0:
                return

            msg = PointCloud2()
            # Unity regex parses `kf_ID` from frame_id
            msg.header.frame_id = f"kf_{kf_id}"
            if ts_ns and ts_ns > 0:
                msg.header.stamp = Time(nanoseconds=int(ts_ns)).to_msg()
            else:
                msg.header.stamp = self.get_clock().now().to_msg()

            msg.height = 1
            msg.width = point_count
            msg.is_bigendian = False
            msg.is_dense = True

            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width
            msg.data = payload

            self.pc_publisher.publish(msg)

            self._count += 1
            now = time.time()
            if now - self._last_log_time >= 1.0:
                self.get_logger().info(
                    f"☁️ Pointcloud received: {point_count} points (count={self._count})"
                )
                self._last_log_time = now
        except Exception as e:
            self.get_logger().error(f"IPC pointcloud handle error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IPCPointCloudReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
