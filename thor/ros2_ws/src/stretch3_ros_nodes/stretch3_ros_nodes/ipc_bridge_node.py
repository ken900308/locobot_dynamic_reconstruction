#!/usr/bin/env python3
"""
ROS2 IPC Bridge Node
- Subscribes: /tf, /tf_static, /locobot/camera/camera/color/image_raw/compressed, /locobot/camera/camera/color/camera_info
- Sends image over UDS IPC socket (binary framing)
"""

import os
import socket
import struct
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo
from tf2_msgs.msg import TFMessage


class IPCBridgeNode(Node):
    MAGIC = b"MSR1"
    VERSION = 1

    # Header: magic(4) version(1) msg_type(1) flags(2) width(4) height(4) encoding(4) timestamp_ns(8) payload_len(4)
    HEADER_STRUCT = struct.Struct("<4sBBHIIIQI")

    MSG_IMAGE = 1
    ENC_JPEG = 4

    def __init__(self):
        super().__init__('ipc_bridge_node')

        self.declare_parameter('socket_path', '/tmp/ipc_socket/locobot/mast3r_image.sock')
        self.declare_parameter('image_topic', '/locobot/camera/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/locobot/camera/camera/color/camera_info')
        self.declare_parameter('qos_reliable', True)

        self.socket_path = self.get_parameter('socket_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        qos_reliable = self.get_parameter('qos_reliable').get_parameter_value().bool_value

        self._sock = None
        self._sock_lock = threading.Lock()

        reliability = ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT
        image_qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )

        tf_static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub_image = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            image_qos,
        )

        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            image_qos,
        )

        self.sub_tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_qos,
        )

        self.sub_tf_static = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            tf_static_qos,
        )

        self.camera_info = None
        self.image_count = 0

        self.get_logger().info(f"📡 IPC socket: {self.socket_path}")
        self.get_logger().info(f"📷 Image topic: {self.image_topic}")
        self.get_logger().info(f"📐 Camera info: {self.camera_info_topic}")

    def _connect(self):
        with self._sock_lock:
            if self._sock is not None:
                return True

            try:
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(self.socket_path)
                self._sock = sock
                return True
            except Exception as e:
                self.get_logger().warn(f"IPC connect failed: {e}")
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass
                self._sock = None
                return False

    def _send_frame(self, width, height, timestamp_ns, payload: bytes):
        if not self._connect():
            return False

        header = self.HEADER_STRUCT.pack(
            self.MAGIC,
            self.VERSION,
            self.MSG_IMAGE,
            0,
            width,
            height,
            self.ENC_JPEG,
            timestamp_ns,
            len(payload),
        )

        with self._sock_lock:
            try:
                self._sock.sendall(header + payload)
                return True
            except Exception as e:
                self.get_logger().warn(f"IPC send failed: {e}")
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                return False

    def image_callback(self, msg: CompressedImage):
        try:
            payload = bytes(msg.data)

            np_arr = np.frombuffer(payload, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                return
            height, width = cv_image.shape[:2]

            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            ok = self._send_frame(width, height, timestamp_ns, payload)

            self.image_count += 1
            if ok and self.image_count % 30 == 1:
                self.get_logger().info(
                    f"✅ Image transferred to socket successfully (count={self.image_count}, {width}x{height})"
                )
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("📐 CameraInfo received")
            self.get_logger().info(f"  Resolution: {msg.width}x{msg.height}")

    def tf_callback(self, msg: TFMessage):
        if len(msg.transforms) > 0:
            self.get_logger().debug(f"/tf received: {len(msg.transforms)} transforms")

    def tf_static_callback(self, msg: TFMessage):
        if len(msg.transforms) > 0:
            self.get_logger().debug(f"/tf_static received: {len(msg.transforms)} transforms")


def main(args=None):
    rclpy.init(args=args)
    node = IPCBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
