#!/usr/bin/env python3
"""
MASt3R-SLAM ROS2 Node with Visualization (Improved Version)

主要改進：
1. 使用短 FIFO (deque) 取代深 queue，避免批次跳幀
2. 視覺化子程序改用 CPU only，避免跨程序 CUDA 問題
3. 更清晰的日誌輸出
"""

import sys
import os
import time
import signal
import threading
import math
from collections import deque  # 改進 1: 使用 deque 作為短 FIFO
import numpy as np
import cv2
import torch
import multiprocessing as mp
from pathlib import Path
import argparse
import datetime
import json

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros

# Add MASt3R paths
sys.path.insert(0, '/workspace/MASt3R-SLAM/thirdparty/mast3r')
sys.path.insert(0, '/workspace/MASt3R-SLAM')

# Global reference for signal handling
_node_instance = None

# Import required modules directly to avoid pickle issues
from main_mast3r import run_backend
from mast3r_slam.evaluate import save_reconstruction

# MASt3R-SLAM imports
from mast3r_slam.config import load_config, config
from mast3r_slam.dataloader import Intrinsics
from mast3r_slam.frame import Mode, SharedKeyframes, SharedStates, create_frame
from mast3r_slam.keyframe_metadata_exporter import KeyframeMetadataExporter
from mast3r_slam.mast3r_utils import load_mast3r, mast3r_inference_mono
from mast3r_slam.multiprocess_utils import new_queue, try_get_msg
from mast3r_slam.tracker import FrameTracker
from mast3r_slam.visualization import WindowMsg, run_visualization
import lietorch

# Global variables for graceful shutdown
should_exit = False

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global should_exit
    should_exit = True
    print("\nReceived interrupt signal. Initiating graceful shutdown...")

def cleanup_signal_handler(sig, frame):
    """Handle cleanup for all signals"""
    global _node_instance, should_exit
    should_exit = True
    
    print(f"\n🛑 Received signal {sig}. Initiating comprehensive cleanup...")
    
    if _node_instance:
        try:
            _node_instance.cleanup_processes()
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    # Force exit after cleanup
    import os
    os._exit(0)

def _run_visualization_cpu(cfg, states, keyframes, main2viz, viz2main):
    """
    改進 2: 視覺化子程序強制使用 CPU，避免跨程序 CUDA 記憶體問題
    """
    import os
    os.environ["CUDA_VISIBLE_DEVICES"] = ""  # 子程序看不到 GPU
    
    # 現在才導入 visualization，確保它在 CPU-only 環境下初始化
    from mast3r_slam.visualization import run_visualization
    run_visualization(cfg, states, keyframes, main2viz, viz2main)

class MASt3RSLAMVisualizationNode(Node):
    """MASt3R-SLAM ROS2 Node with visualization for monitoring inference"""
    
    def __init__(self):
        super().__init__('mast3r_slam_node')

        # Set global reference for signal handling
        global _node_instance
        _node_instance = self

        # Initialize SLAM state early to avoid AttributeError
        self.slam_initialized = False
        self.manager = None
        self.keyframes = None
        self.states = None

        # Initialize parameters
        self.declare_parameter('config_file', 'config/base.yaml')
        self.declare_parameter('save_as', 'stretch3_slam')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('enable_visualization', False)  # 預設關閉視覺化
        self.declare_parameter('max_fps', 15.0)  # 新增：最大處理 FPS
        self.declare_parameter('use_compressed', False)  # 新增：使用壓縮圖像
        self.declare_parameter('use_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.0.60')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('rosbridge_tf_topic', '/tf')
        self.declare_parameter('rosbridge_tf_static_topic', '/locobot/tf_static_relay')
        self.declare_parameter('frame_pointcloud_topic', '/mast3r/frame_pointcloud')
        self.declare_parameter('fullmap_pointcloud_topic', '/mast3r/pointcloud_in_map')
        default_raw_topic = f"/{os.environ.get('ROBOT_ID', 'robot1')}/mast3r/pointcloud_in_mast3r_map"
        self.declare_parameter('fullmap_raw_pointcloud_topic', default_raw_topic)

        # Get parameters
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.save_as = self.get_parameter('save_as').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        self.use_rosbridge = self.get_parameter('use_rosbridge').get_parameter_value().bool_value
        self.rosbridge_host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.rosbridge_port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.rosbridge_tf_topic = self.get_parameter('rosbridge_tf_topic').get_parameter_value().string_value
        self.rosbridge_tf_static_topic = self.get_parameter('rosbridge_tf_static_topic').get_parameter_value().string_value
        self.frame_pointcloud_topic = self.get_parameter('frame_pointcloud_topic').get_parameter_value().string_value
        self.fullmap_pointcloud_topic = self.get_parameter('fullmap_pointcloud_topic').get_parameter_value().string_value
        self.fullmap_raw_pointcloud_topic = self.get_parameter('fullmap_raw_pointcloud_topic').get_parameter_value().string_value

        # 自動偵測或使用參數設定壓縮模式
        use_compressed_param = self.get_parameter('use_compressed').get_parameter_value().bool_value
        self.use_compressed = use_compressed_param or '/compressed' in self.image_topic

        self.bridge = CvBridge()
        self.robot_id = os.environ.get('ROBOT_ID', 'robot1')
        self.keyframe_metadata_exporter = None
        
        # 改進 1: 使用短 FIFO (deque) 取代深 queue
        # maxlen=3 確保只保留最近的 3 張影像，避免延遲堆積
        self.image_buffer = deque(maxlen=3)
        self.buffer_lock = threading.Lock()
        
        # FPS 控制（可選的額外節流）
        self.min_frame_interval = 1.0 / self.max_fps if self.max_fps > 0 else 0
        self.last_frame_time = 0
        
        # Camera info and processing state
        self.camera_info = None
        self.processing_thread = None
        self.is_processing = False
        
        # Statistics
        self.image_count = 0
        self.dropped_count = 0  # 新增：追蹤丟棄的幀數
        self.last_timestamp = None
        self.processed_count = 0
        self.start_time = time.time()
        
        # Thread safety locks
        self.keyframes_lock = threading.Lock()
        self.states_lock = threading.Lock()
        self.save_lock = threading.Lock()  # 改進 3: 防止儲存重入
        self.last_keyframe_count = 0
        
        # 週期性儲存管理
        self.save_interval = 5.0  # 每 5 秒儲存一次
        self.last_save_time = 0
        
        # QoS profiles and subscriptions
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self.use_rosbridge:
            self.get_logger().info(
                f"🌐 Using rosbridge for robot topics: {self.rosbridge_host}:{self.rosbridge_port}"
            )
            self.get_logger().info(f"   image: {self.image_topic}")
            self.get_logger().info(f"   camera_info: {self.camera_info_topic}")
            self.get_logger().info(f"   tf: {self.rosbridge_tf_topic}")
            self.get_logger().info(f"   tf_static: {self.rosbridge_tf_static_topic}")
            self.ros_client = None
            self.rosbridge_listener = None
            self.rosbridge_camera_info_listener = None
            self.rosbridge_tf_listener = None
            self.rosbridge_tf_static_listener = None
            self._rosbridge_image_queue = []
            self._rosbridge_camera_info_queue = []
            self._rosbridge_tf_queue = []
            self._rosbridge_tf_static_queue = []
            self._static_tf_cache = {}
            self._rosbridge_tf_count = 0
            self._rosbridge_tf_static_count = 0
            self._rosbridge_image_received_count = 0
            self._rosbridge_image_overwrite_count = 0
            self._last_rosbridge_image_stamp = None
            self._last_rosbridge_image_wall_time = None
            self._last_tf_log_time = 0
            self._rosbridge_queue_lock = threading.Lock()
            self._rosbridge_camera_info_queue_lock = threading.Lock()
            self._rosbridge_tf_queue_lock = threading.Lock()
            self._rosbridge_tf_static_queue_lock = threading.Lock()
            self.init_rosbridge_in_main_thread()
        else:
            if self.use_compressed:
                self.get_logger().info("🗜️ Using CompressedImage subscription")
                self.image_sub = self.create_subscription(
                    CompressedImage,
                    self.image_topic,
                    self.compressed_image_callback,
                    image_qos,
                )
            else:
                self.get_logger().info("📷 Using raw Image subscription")
                self.image_sub = self.create_subscription(
                    Image,
                    self.image_topic,
                    self.image_callback,
                    image_qos,
                )

            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                image_qos,
            )

        # Timer for stats
        self.stats_timer = self.create_timer(5.0, self.print_stats)

        # Timer for periodic saving (disabled)
        # self.save_timer = self.create_timer(self.save_interval, self.periodic_save_callback)

        # 新增：逐幀點雲發布器 (Frame-by-frame PointCloud2 publisher)
        frame_pc_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.frame_pc_publisher = self.create_publisher(
            PointCloud2,
            self.frame_pointcloud_topic,
            frame_pc_qos,
        )

        # [IBGR Chunking] 全域地圖分塊直達 publisher
        # 全域地圖的點已在 Python 端用 T_WC 轉換為世界座標，
        # 不需要再經過 pc2_to_map（pc2_to_map 的 10Hz FPS 限制 + depth=1 會 drop 掉幾乎所有 chunk）
        fullmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fullmap_pc_publisher = self.create_publisher(
            PointCloud2,
            self.fullmap_pointcloud_topic,
            fullmap_qos,
        )
        self.fullmap_raw_pc_publisher = self.create_publisher(
            PointCloud2,
            self.fullmap_raw_pointcloud_topic,
            fullmap_qos,
        )
        self.get_logger().info(f"🎯 Frame PointCloud publisher: {self.frame_pointcloud_topic}")
        self.get_logger().info(
            f"🎯 Full Map direct publisher:  {self.fullmap_pointcloud_topic} (bypass pc2_to_map)"
        )
        self.get_logger().info(
            f"🎯 Raw Full Map publisher:     {self.fullmap_raw_pointcloud_topic} (ROS/MASt3R frame)"
        )
        self.keyframe_metadata_exporter = KeyframeMetadataExporter(self, self.robot_id)
        self.min_confidence_threshold = 0.95  # 置信度過濾閾值
        self.pc_publish_count = 0
        self.pc_last_log_time = time.time()
        self.loop_closure_count = 0  # [新增] 紀錄 PGO 觸發全域重繪的次數
        self.fullmap_revision = 0
        self.mast3r_stage_fps_ema = 0.0
        self.mast3r_stage_fps_alpha = 0.1

        self.get_logger().info(f"MASt3R-SLAM Node with Visualization initialized")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"Save as: {self.save_as}")
        self.get_logger().info(f"Max FPS: {self.max_fps}")
        self.get_logger().info(f"Image buffer size: 3 (short FIFO)")
        
        # 改進 2: 根據參數正確顯示視覺化狀態
        if self.enable_visualization:
            self.get_logger().info("🎥 Visualization ENABLED - You can monitor the SLAM process")
        else:
            self.get_logger().info("🖥️ Visualization DISABLED (headless mode)")
        
        # Initialize MASt3R-SLAM components
        self.initialize_slam()

    def _normalize_frame_id(self, frame_id: str) -> str:
        if frame_id is None:
            return ''
        normalized = str(frame_id).strip()
        if normalized.startswith('/'):
            normalized = normalized.lstrip('/')
        return normalized

    def init_rosbridge_in_main_thread(self):
        """Subscribe robot-facing topics through rosbridge and rebroadcast TF locally."""
        try:
            import roslibpy
            self._roslibpy = roslibpy
        except ImportError:
            self.get_logger().error("roslibpy not installed; install it in the container setup")
            return

        try:
            self.get_logger().info(f"Connecting to rosbridge at {self.rosbridge_host}:{self.rosbridge_port}")
            self.ros_client = roslibpy.Ros(host=self.rosbridge_host, port=self.rosbridge_port)
            self.ros_client.run()

            for _ in range(50):
                if self.ros_client.is_connected:
                    break
                time.sleep(0.1)

            if not self.ros_client.is_connected:
                self.get_logger().error("Failed to connect to rosbridge")
                return

            image_topic = self.image_topic
            image_type = 'sensor_msgs/CompressedImage' if self.use_compressed else 'sensor_msgs/Image'
            if self.use_compressed and not image_topic.endswith('/compressed'):
                image_topic = image_topic + '/compressed'

            self.rosbridge_listener = roslibpy.Topic(self.ros_client, image_topic, image_type)
            self.rosbridge_listener.subscribe(self._queue_latest_rosbridge_image_msg)
            self.get_logger().info(f"Subscribed to {image_topic} via rosbridge")

            self.rosbridge_camera_info_listener = roslibpy.Topic(
                self.ros_client,
                self.camera_info_topic,
                'sensor_msgs/CameraInfo'
            )
            self.rosbridge_camera_info_listener.subscribe(
                lambda msg: self._queue_rosbridge_msg(
                    self._rosbridge_camera_info_queue_lock,
                    self._rosbridge_camera_info_queue,
                    msg
                )
            )
            self.get_logger().info(f"Subscribed to {self.camera_info_topic} via rosbridge")

            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

            self.rosbridge_tf_listener = roslibpy.Topic(
                self.ros_client,
                self.rosbridge_tf_topic,
                'tf2_msgs/TFMessage',
                queue_size=100,
                queue_length=1
            )
            self.rosbridge_tf_listener.subscribe(
                lambda msg: self._queue_rosbridge_msg(self._rosbridge_tf_queue_lock, self._rosbridge_tf_queue, msg)
            )
            self.get_logger().info(f"Subscribed to {self.rosbridge_tf_topic} via rosbridge")

            self.rosbridge_tf_static_listener = roslibpy.Topic(
                self.ros_client,
                self.rosbridge_tf_static_topic,
                'tf2_msgs/TFMessage',
                latch=True,
                queue_size=100,
                queue_length=1
            )
            self.rosbridge_tf_static_listener.subscribe(
                lambda msg: self._queue_rosbridge_msg(
                    self._rosbridge_tf_static_queue_lock,
                    self._rosbridge_tf_static_queue,
                    msg
                )
            )
            self.get_logger().info(f"Subscribed to {self.rosbridge_tf_static_topic} via rosbridge")

            self.rosbridge_poll_timer = self.create_timer(0.05, self.poll_rosbridge_queue)
            self.static_tf_republish_timer = self.create_timer(1.0, self.republish_static_tf_cache)
        except Exception as e:
            self.get_logger().error(f"Rosbridge init failed: {e}")

    def _queue_rosbridge_msg(self, lock, queue, msg):
        with lock:
            queue.append(msg)

    def _queue_latest_rosbridge_image_msg(self, msg):
        # Keep only the latest robot image. Processing old images creates visible lag
        # and can make keyframe updates appear minutes late when SLAM is slower than the camera stream.
        with self._rosbridge_queue_lock:
            if self._rosbridge_image_queue:
                self._rosbridge_image_overwrite_count += len(self._rosbridge_image_queue)
                self._rosbridge_image_queue.clear()
            self._rosbridge_image_queue.append(msg)
            self._rosbridge_image_received_count += 1

    def poll_rosbridge_queue(self):
        image_msg = None
        with self._rosbridge_queue_lock:
            if self._rosbridge_image_queue:
                image_msg = self._rosbridge_image_queue[-1]
                self._rosbridge_image_queue.clear()
        if image_msg is not None:
            self.process_rosbridge_image(image_msg)

        with self._rosbridge_camera_info_queue_lock:
            if self._rosbridge_camera_info_queue:
                msg = self._rosbridge_camera_info_queue.pop(0)
                self.process_rosbridge_camera_info(msg)

        with self._rosbridge_tf_queue_lock:
            while self._rosbridge_tf_queue:
                self.process_rosbridge_tf(self._rosbridge_tf_queue.pop(0), is_static=False)

        with self._rosbridge_tf_static_queue_lock:
            while self._rosbridge_tf_static_queue:
                self.process_rosbridge_tf(self._rosbridge_tf_static_queue.pop(0), is_static=True)

    def process_rosbridge_tf(self, msg, is_static=False):
        try:
            transforms = msg.get('transforms', [])
            ros2_transforms = []
            current_time = self.get_clock().now().to_msg()

            for tf_msg in transforms:
                t = TransformStamped()
                header = tf_msg.get('header', {})
                if is_static:
                    t.header.stamp.sec = 0
                    t.header.stamp.nanosec = 0
                else:
                    t.header.stamp = current_time

                parent_frame = self._normalize_frame_id(header.get('frame_id', ''))
                child_frame = self._normalize_frame_id(tf_msg.get('child_frame_id', ''))
                if not parent_frame or not child_frame:
                    continue
                t.header.frame_id = parent_frame
                t.child_frame_id = child_frame

                transform = tf_msg.get('transform', {})
                translation = transform.get('translation', {})
                rotation = transform.get('rotation', {})
                t.transform.translation.x = float(translation.get('x', 0.0))
                t.transform.translation.y = float(translation.get('y', 0.0))
                t.transform.translation.z = float(translation.get('z', 0.0))
                t.transform.rotation.x = float(rotation.get('x', 0.0))
                t.transform.rotation.y = float(rotation.get('y', 0.0))
                t.transform.rotation.z = float(rotation.get('z', 0.0))
                t.transform.rotation.w = float(rotation.get('w', 1.0))
                ros2_transforms.append(t)

            if ros2_transforms:
                if is_static:
                    for t in ros2_transforms:
                        self._static_tf_cache[(t.header.frame_id, t.child_frame_id)] = t
                    self._rosbridge_tf_static_count += len(ros2_transforms)
                    self.static_tf_broadcaster.sendTransform(list(self._static_tf_cache.values()))
                else:
                    self._rosbridge_tf_count += len(ros2_transforms)
                    self.tf_broadcaster.sendTransform(ros2_transforms)

                now = time.time()
                if now - self._last_tf_log_time >= 5.0:
                    self.get_logger().info(
                        f"Rosbridge TF relay: dynamic={self._rosbridge_tf_count}, "
                        f"static={self._rosbridge_tf_static_count}, "
                        f"static_cache={len(self._static_tf_cache)}"
                    )
                    self._last_tf_log_time = now
        except Exception as e:
            self.get_logger().error(f"Error processing rosbridge TF: {e}")

    def republish_static_tf_cache(self):
        if not getattr(self, 'use_rosbridge', False):
            return
        if not hasattr(self, '_static_tf_cache') or not self._static_tf_cache:
            return
        try:
            self.static_tf_broadcaster.sendTransform(list(self._static_tf_cache.values()))
        except Exception as e:
            self.get_logger().warn(f"Failed to republish cached static TF: {e}")

    def process_rosbridge_camera_info(self, msg):
        if self.camera_info is not None:
            return
        try:
            info = CameraInfo()
            header = msg.get('header', {})
            stamp = header.get('stamp', {})
            info.header.stamp.sec = int(stamp.get('sec', 0)) if isinstance(stamp, dict) else 0
            info.header.stamp.nanosec = int(stamp.get('nanosec', stamp.get('nsecs', 0))) if isinstance(stamp, dict) else 0
            info.header.frame_id = self._normalize_frame_id(header.get('frame_id', ''))
            info.height = int(msg.get('height', 0))
            info.width = int(msg.get('width', 0))
            info.distortion_model = msg.get('distortion_model', '')
            info.d = [float(v) for v in msg.get('d', msg.get('D', []))]
            k = msg.get('k', msg.get('K', []))
            r = msg.get('r', msg.get('R', []))
            pp = msg.get('p', msg.get('P', []))
            if len(k) == 9:
                info.k = [float(v) for v in k]
            if len(r) == 9:
                info.r = [float(v) for v in r]
            if len(pp) == 12:
                info.p = [float(v) for v in pp]
            info.binning_x = int(msg.get('binning_x', 0))
            info.binning_y = int(msg.get('binning_y', 0))
            self.camera_info_callback(info)
        except Exception as e:
            self.get_logger().error(f"Error processing rosbridge CameraInfo: {e}")

    def process_rosbridge_image(self, msg):
        try:
            import base64
            current_time = time.time()
            if self.min_frame_interval > 0:
                if current_time - self.last_frame_time < self.min_frame_interval:
                    self.dropped_count += 1
                    return
                self.last_frame_time = current_time

            data_base64 = msg.get('data', '')
            image_data = base64.b64decode(data_base64)

            if self.use_compressed:
                np_arr = np.frombuffer(image_data, dtype=np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if cv_image is None:
                    self.get_logger().warning("Failed to decode rosbridge compressed image")
                    return
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                width = int(msg['width'])
                height = int(msg['height'])
                encoding = msg.get('encoding', 'rgb8')
                if encoding in ['rgb8', 'bgr8']:
                    cv_image = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 3)
                    if encoding == 'bgr8':
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                elif encoding == 'rgba8':
                    arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
                    cv_image = cv2.cvtColor(arr, cv2.COLOR_RGBA2RGB)
                elif encoding == 'mono8':
                    arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width)
                    cv_image = cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
                else:
                    self.get_logger().warning(f"Unsupported rosbridge image encoding: {encoding}")
                    return

            cv_image = cv_image.astype(np.float32) / 255.0
            stamp = msg.get('header', {}).get('stamp', {})
            sec = stamp.get('sec', 0) if isinstance(stamp, dict) else 0
            nanosec = stamp.get('nanosec', stamp.get('nsecs', 0)) if isinstance(stamp, dict) else 0
            timestamp = sec + nanosec * 1e-9
            if timestamp == 0:
                timestamp = time.time()

            self._last_rosbridge_image_stamp = timestamp
            self._last_rosbridge_image_wall_time = time.time()

            with self.buffer_lock:
                old_size = len(self.image_buffer)
                self.image_buffer.append((timestamp, cv_image))
                if old_size == 3:
                    self.dropped_count += 1
                    if self.dropped_count % 50 == 0:
                        self.get_logger().info(f"Dropped {self.dropped_count} frames due to processing lag")

            self.image_count += 1
            if self.image_count == 1:
                mode = 'compressed' if self.use_compressed else 'raw'
                self.get_logger().info(f"First image received via rosbridge ({mode}), shape={cv_image.shape}")
        except Exception as e:
            self.get_logger().error(f"Error processing rosbridge image: {e}")

    def cleanup_rosbridge(self):
        for attr in [
            'rosbridge_listener',
            'rosbridge_camera_info_listener',
            'rosbridge_tf_listener',
            'rosbridge_tf_static_listener',
        ]:
            listener = getattr(self, attr, None)
            if listener is not None:
                try:
                    listener.unsubscribe()
                except Exception:
                    pass
        if getattr(self, 'ros_client', None) is not None:
            try:
                self.ros_client.terminate()
            except Exception:
                pass

    def camera_info_callback(self, msg):
        """Store camera intrinsics from ROS CameraInfo message"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera intrinsics received!")
            K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"  Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"  Intrinsics: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
        
    def image_callback(self, msg):
        """
        改進 1: 使用非阻塞的短 FIFO 處理影像
        """
        try:
            if msg is None:
                self.get_logger().warning("Received None message")
                return
                
            if not hasattr(msg, 'data') or msg.data is None:
                self.get_logger().warning("Message has no data")
                return
            
            # FPS 節流（可選）
            current_time = time.time()
            if self.min_frame_interval > 0:
                if current_time - self.last_frame_time < self.min_frame_interval:
                    # 跳過太頻繁的幀
                    self.dropped_count += 1
                    return
                self.last_frame_time = current_time
            
            # Convert raw image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            if cv_image is None:
                self.get_logger().warning("Failed to convert raw image")
                return
            
            
            # 註：D435i 直立時需要旋轉 90 度，D415 橫放則不需要
            # cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Convert to 0-1 range for MASt3R
            if cv_image.dtype == np.uint8:
                cv_image = cv_image.astype(np.float32) / 255.0
            
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 檢查時間戳穩定性
            if hasattr(self, 'last_timestamp') and self.last_timestamp is not None:
                dt = timestamp - self.last_timestamp
                if dt > 0.5:  # 超過 0.5 秒的跳躍
                    self.get_logger().warning(f"⚠️ Large timestamp jump: {dt:.3f}s")
                elif dt < 0:  # 時間倒退
                    self.get_logger().warning(f"⚠️ Timestamp went backwards: {dt:.3f}s")
            self.last_timestamp = timestamp
            
            # 改進 1: 使用非阻塞的 deque append
            # 如果 buffer 滿了，自動丟棄最舊的影像
            with self.buffer_lock:
                old_size = len(self.image_buffer)
                self.image_buffer.append((timestamp, cv_image))
                if old_size == 3:  # maxlen=3，滿了會自動丟棄最舊的
                    self.dropped_count += 1
                    if self.dropped_count % 50 == 0:  # 每 50 幀報告一次
                        self.get_logger().info(f"📊 Dropped {self.dropped_count} frames due to processing lag")
            
            self.image_count += 1
            
            if self.image_count == 1:
                self.get_logger().info(f"🎉 First image received!")
                self.get_logger().info(f"  Shape: {cv_image.shape}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def compressed_image_callback(self, msg):
        """處理 CompressedImage 訊息"""
        try:
            if msg is None or msg.data is None:
                self.get_logger().warning("Received None compressed message")
                return
            
            # FPS 節流
            current_time = time.time()
            if self.min_frame_interval > 0:
                if current_time - self.last_frame_time < self.min_frame_interval:
                    self.dropped_count += 1
                    return
                self.last_frame_time = current_time
            
            # 解碼壓縮圖像 (JPEG/PNG)
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warning(f"Failed to decode compressed image (format: {msg.format})")
                return
            
            # BGR -> RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Convert to 0-1 range for MASt3R
            cv_image = cv_image.astype(np.float32) / 255.0
            
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 使用非阻塞的 deque append
            with self.buffer_lock:
                old_size = len(self.image_buffer)
                self.image_buffer.append((timestamp, cv_image))
                if old_size == 3:
                    self.dropped_count += 1
                    if self.dropped_count % 50 == 0:
                        self.get_logger().info(f"📊 Dropped {self.dropped_count} frames due to processing lag")
            
            self.image_count += 1
            
            if self.image_count == 1:
                self.get_logger().info(f"🎉 First compressed image received!")
                self.get_logger().info(f"  Format: {msg.format}")
                self.get_logger().info(f"  Shape: {cv_image.shape}")
                
        except Exception as e:
            import traceback
            self.get_logger().error(f"Error processing compressed image: {str(e)}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
    
    def initialize_slam(self):
        """Initialize MASt3R-SLAM components with improved visualization"""
        global should_exit
        
        try:
            # Load configuration
            load_config(self.config_file)
            self.get_logger().info(f"✅ Config loaded: {self.config_file}")
            
            # Set up multiprocessing
            mp.set_start_method("spawn", force=True)
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.set_grad_enabled(False)
            
            # Wait for camera info with timeout
            self.get_logger().info("⏳ Waiting for camera intrinsics...")
            timeout_start = time.time()
            while self.camera_info is None and not should_exit:
                if time.time() - timeout_start > 10.0:  # 10 second timeout
                    self.get_logger().warn("⚠️ Camera info timeout - proceeding with default settings")
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if should_exit:
                return
                
            # Set up camera intrinsics
            self.raw_h, self.raw_w = 480, 640  # D415 橫放: 640x480 (width x height)
            if self.camera_info:
                self.setup_camera_intrinsics()  # ROS 模式：從 CameraInfo 讀取並正確縮放
            if config.get('use_calib', False) and not hasattr(self, 'K'):
                raise RuntimeError('use_calib=True but no valid CameraInfo intrinsics were received')
            
            # We'll initialize SLAM with correct dimensions after first image processing
            self.slam_initialized = False
            self.manager = None
            self.keyframes = None
            self.states = None
            
            # Load MASt3R model
            self.get_logger().info("🧠 Loading MASt3R model...")
            self.model = load_mast3r(device=self.device)
            self.model.share_memory()
            self.get_logger().info("✅ MASt3R model loaded")
            
            # Create a dataset-like object for saving
            class SimpleDataset:
                def __init__(self):
                    import pathlib
                    import time
                    self.save_results = True
                    self.timestamps = []
                    self.dataset_path = pathlib.Path("stretch3_ros2_slam")
                    self.start_time = time.time()
                    
                def get_timestamps_for_keyframes(self, num_keyframes):
                    """Generate timestamps for keyframes based on current time"""
                    max_frame_id = 0
                    try:
                        if hasattr(self, '_parent_keyframes') and self._parent_keyframes:
                            for i in range(len(self._parent_keyframes)):
                                kf = self._parent_keyframes[i]
                                if hasattr(kf, 'frame_id'):
                                    max_frame_id = max(max_frame_id, kf.frame_id)
                    except Exception as e:
                        max_frame_id = num_keyframes * 100
                    
                    needed_timestamps = max(max_frame_id + 10, num_keyframes)
                    
                    while len(self.timestamps) < needed_timestamps:
                        next_idx = len(self.timestamps)
                        self.timestamps.append(self.start_time + next_idx * 0.033)  # ~30 FPS intervals
                    
                    return self.timestamps
                    
            self.dataset = SimpleDataset()
            
            # Initialize default window message
            self.last_msg = WindowMsg()
            
            # Tracker will be initialized after SLAM setup
            self.tracker = None
            
            # Processes will be started after SLAM initialization
            self.viz_process = None
            self.backend_process = None
            
            # Start processing thread
            self.is_processing = True
            self.processing_thread = threading.Thread(target=self.process_images)
            self.processing_thread.start()
            self.get_logger().info("🚀 SLAM processing thread started - waiting for first image...")
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"Failed to initialize SLAM: {str(e)}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            raise
    
    def setup_camera_intrinsics(self):
        """Set up camera intrinsics from ROS CameraInfo (with correct K_frame scaling for MASt3R 512px)"""
        from mast3r_slam.dataloader import Intrinsics
        K_matrix = np.array(self.camera_info.k).reshape(3, 3)
        W = self.camera_info.width
        H = self.camera_info.height
        calib = [K_matrix[0, 0], K_matrix[1, 1], K_matrix[0, 2], K_matrix[1, 2]]
        
        # Intrinsics.from_calib 會計算 K_frame：把 640x480 的 K 縮放到 MASt3R 的 512px 大小
        intrinsics = Intrinsics.from_calib(512, W, H, calib)
        if intrinsics is None:
            self.get_logger().warn("⚠️ Intrinsics.from_calib returned None (use_calib=False in config?)")
            return
        
        # K_frame 是對 MASt3R 縮放後圖像正確的內參矩陣
        self.K = torch.from_numpy(intrinsics.K_frame).to(self.device, dtype=torch.float32)
        
        self.get_logger().info(f"📐 Camera intrinsics (ROS CameraInfo, scaled to MASt3R 512px):")
        self.get_logger().info(f"  Raw: {W}x{H}, fx={calib[0]:.2f}, fy={calib[1]:.2f}, cx={calib[2]:.2f}, cy={calib[3]:.2f}")
        self.get_logger().info(f"  K_frame: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def publish_keyframe_pointcloud(self):
        """
        發布最新 keyframe 的點雲到 /mast3r/frame_pointcloud

        Keyframe 有經過：
        - Fusion (weighted_pointmap averaging)
        - 優化後的姿態 (global optimization)
        """
        try:
            with self.keyframes_lock:
                if self.keyframes is None or len(self.keyframes) == 0:
                    return
                keyframe = self.keyframes.last_keyframe()
                if keyframe is None or keyframe.X_canon is None or keyframe.C is None:
                    return

                X_canon = keyframe.X_canon
                C = keyframe.C
                T_WC = keyframe.T_WC

            valid_mask = (C[:, 0] > self.min_confidence_threshold).cpu().numpy()
            if valid_mask.sum() == 0:
                return

            # 轉換到 MASt3R 世界座標系。
            X_world = T_WC.act(X_canon)
            points_np = X_world.detach().cpu().numpy()[valid_mask].astype(np.float32)

            if hasattr(keyframe, 'uimg') and keyframe.uimg is not None:
                uimg = keyframe.uimg.numpy() if hasattr(keyframe.uimg, 'numpy') else keyframe.uimg
                flat_colors = (uimg.reshape(-1, 3) * 255).astype(np.uint8)
                colors_np = flat_colors[valid_mask]
            else:
                colors_np = np.ones((points_np.shape[0], 3), dtype=np.uint8) * 255

            if len(points_np) == 0:
                return

            # Unity 左手座標轉換：Z = -Z (ROS 右手 → Unity 左手)
            points_np[:, 2] = -points_np[:, 2]

            point_dtype = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('rgb', '<u4'),
            ])
            point_data = np.zeros(len(points_np), dtype=point_dtype)
            point_data['x'] = points_np[:, 0]
            point_data['y'] = points_np[:, 1]
            point_data['z'] = points_np[:, 2]

            r = colors_np[:, 0].astype(np.uint32)
            g = colors_np[:, 1].astype(np.uint32)
            b = colors_np[:, 2].astype(np.uint32)
            point_data['rgb'] = (r << 16) | (g << 8) | b

            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"kf_{keyframe.frame_id}"
            msg.height = 1
            msg.width = len(points_np)
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
            msg.data = point_data.tobytes()

            self.frame_pc_publisher.publish(msg)
            if self.keyframe_metadata_exporter is not None:
                self.keyframe_metadata_exporter.publish(keyframe)

            self.pc_publish_count += 1
            now = time.time()
            if now - self.pc_last_log_time >= 1.0:
                self.get_logger().info(
                    f"☁️ Published frame pointcloud: {len(points_np)} points "
                    f"(count={self.pc_publish_count})"
                )
                self.pc_last_log_time = now

        except Exception as e:
            self.get_logger().error(f"❌ Error publishing frame pointcloud: {str(e)}")

    def _make_fullmap_point_data(self, points_np, colors_np):
        point_dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        point_data = np.zeros(len(points_np), dtype=point_dtype)
        point_data['x'] = points_np[:, 0]
        point_data['y'] = points_np[:, 1]
        point_data['z'] = points_np[:, 2]

        r = colors_np[:, 0].astype(np.uint32)
        g = colors_np[:, 1].astype(np.uint32)
        b = colors_np[:, 2].astype(np.uint32)
        point_data['rgb'] = (r << 16) | (g << 8) | b
        return point_data

    def _publish_chunked_fullmap(self, publisher, point_data, frame_id_prefix, stamp):
        if publisher is None:
            return 0

        CHUNK_SIZE = 200_000
        POINT_BYTES = 16
        num_points = len(point_data)
        total_chunks = (num_points + CHUNK_SIZE - 1) // CHUNK_SIZE
        payload = point_data.tobytes()

        for chunk_idx in range(total_chunks):
            start = chunk_idx * CHUNK_SIZE
            end = min(start + CHUNK_SIZE, num_points)
            chunk_count = end - start
            chunk_payload = payload[start * POINT_BYTES : end * POINT_BYTES]

            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = stamp
            msg.header.frame_id = f"{frame_id_prefix}_{chunk_idx + 1}_{total_chunks}"
            msg.height = 1
            msg.width = chunk_count
            msg.is_bigendian = False
            msg.is_dense = True
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.point_step = POINT_BYTES
            msg.row_step = POINT_BYTES * chunk_count
            msg.data = list(chunk_payload)
            publisher.publish(msg)
            time.sleep(0.01)

        return total_chunks

    def _republish_keyframe_metadata_only(self, keyframes, reason="local_pgo"):
        if self.keyframe_metadata_exporter is None:
            return
        count = 0
        for keyframe in keyframes:
            self.keyframe_metadata_exporter.publish_metadata_only(keyframe, reason=reason)
            count += 1
        if count:
            self.get_logger().info(f"Republished metadata-only poses for {count} keyframes ({reason})")

    def publish_full_map_pointcloud(self):
        """發布所有 keyframes 的全域點雲；只在後端 PGO 更新後送出。"""
        try:
            with self.keyframes_lock:
                if self.keyframes is None or len(self.keyframes) == 0:
                    return
                if not self.keyframes.check_and_clear_pgo_flag():
                    return

                self.fullmap_revision += 1
                revision = self.fullmap_revision
                stamp = self.get_clock().now().to_msg()
                all_points = []
                all_colors = []
                metadata_keyframes = []
                for idx in range(len(self.keyframes)):
                    keyframe = self.keyframes[idx]
                    metadata_keyframes.append(keyframe)
                    if keyframe.X_canon is None or keyframe.C is None:
                        continue

                    valid_mask = (keyframe.C[:, 0] > self.min_confidence_threshold).cpu().numpy()
                    if valid_mask.sum() == 0:
                        continue

                    X_world = keyframe.T_WC.act(keyframe.X_canon)
                    points_np = X_world.detach().cpu().numpy()[valid_mask].astype(np.float32)

                    if hasattr(keyframe, 'uimg') and keyframe.uimg is not None:
                        uimg = keyframe.uimg.numpy() if hasattr(keyframe.uimg, 'numpy') else keyframe.uimg
                        flat_colors = (uimg.reshape(-1, 3) * 255).astype(np.uint8)
                        colors_np = flat_colors[valid_mask]
                    else:
                        colors_np = np.ones((points_np.shape[0], 3), dtype=np.uint8) * 255

                    all_points.append(points_np)
                    all_colors.append(colors_np)

            self._republish_keyframe_metadata_only(metadata_keyframes)
            if len(all_points) == 0:
                return

            merged_points = np.concatenate(all_points, axis=0)
            merged_colors = np.concatenate(all_colors, axis=0)

            max_points = 10_000_000
            if len(merged_points) > max_points:
                indices = np.random.choice(len(merged_points), max_points, replace=False)
                merged_points = merged_points[indices]
                merged_colors = merged_colors[indices]

            raw_point_data = self._make_fullmap_point_data(merged_points, merged_colors)
            raw_chunks = self._publish_chunked_fullmap(
                self.fullmap_raw_pc_publisher,
                raw_point_data,
                f"fullmap_raw_{revision}",
                stamp,
            )

            unity_points = merged_points.copy()
            unity_points[:, 2] = -unity_points[:, 2]
            unity_point_data = self._make_fullmap_point_data(unity_points, merged_colors)
            unity_chunks = self._publish_chunked_fullmap(
                self.fullmap_pc_publisher,
                unity_point_data,
                "kf_999999",
                stamp,
            )

            self.loop_closure_count += 1
            self.get_logger().info(
                f"💥 [Loop Closure PGO #{self.loop_closure_count}] "
                f"Full maps sent: {len(merged_points)} pts, raw_chunks={raw_chunks}, unity_chunks={unity_chunks}."
            )

        except Exception as e:
            self.get_logger().error(f"❌ Error publishing full map pointcloud: {str(e)}")

    def process_images(self):
        """
        改進 1: 使用短 FIFO 處理，確保處理連續的幀
        """
        global should_exit
        
        i = 0
        fps_timer = time.time()
        consecutive_empty = 0  # 追蹤連續空 buffer 次數
        
        self.get_logger().info("🔄 Starting SLAM processing loop...")
        
        while True:
            try:
                # Check for graceful shutdown signal
                if should_exit:
                    self.get_logger().info("🛑 Graceful shutdown initiated...")
                    break
                
                # Check SLAM state and visualization messages
                if self.slam_initialized and hasattr(self, 'states') and self.states:
                    mode = self.states.get_mode()
                    msg = try_get_msg(self.viz2main) if hasattr(self, 'viz2main') else None
                    if msg is not None:
                        self.last_msg = msg
                    
                    if hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_terminated:
                        self.get_logger().info("🛑 Termination signal from visualization")
                        self.states.set_mode(Mode.TERMINATED)
                        break

                    if hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_paused and not self.last_msg.next:
                        self.states.pause()
                        time.sleep(0.01)
                        continue

                    if hasattr(self, 'last_msg') and self.last_msg and not self.last_msg.is_paused:
                        self.states.unpause()
                
                # 改進 1: 從短 FIFO 取出最舊的影像處理
                timestamp = None
                img = None
                with self.buffer_lock:
                    if self.image_buffer:
                        timestamp, img = self.image_buffer.popleft()  # 取最舊的
                        consecutive_empty = 0
                    else:
                        consecutive_empty += 1
                
                if img is None:
                    # 沒有新影像，短暫等待
                    if consecutive_empty > 100:  # 約 1 秒沒有影像
                        if consecutive_empty % 100 == 0:  # 每秒報告一次
                            self.get_logger().info("⏳ Waiting for images...")
                    time.sleep(0.01)
                    continue
                
                # Get current mode for processing
                mode = self.states.get_mode() if (self.slam_initialized and hasattr(self, 'states') and self.states) else Mode.INIT
                
                if should_exit:
                    break
                
                # Initialize SLAM with correct dimensions from first frame
                if not self.slam_initialized:
                    if i == 0:
                        self.get_logger().info("▶️ Starting algorithm pipeline")
                    mast3r_size = 512  # MASt3R standard size
                    
                    # Process first image through MASt3R to get actual dimensions
                    temp_frame = create_frame(0, img, np.eye(4), img_size=mast3r_size, device=self.device)
                    frame_shape = temp_frame.img.shape
                    self.get_logger().info(f"📐 Frame tensor shape: {frame_shape}")
                    
                    # Extract dimensions correctly from tensor shape
                    if len(frame_shape) == 4:  # Batch format: [B, C, H, W]
                        actual_h, actual_w = frame_shape[2], frame_shape[3]
                    elif len(frame_shape) == 3:  # CHW format: [C, H, W]
                        actual_h, actual_w = frame_shape[1], frame_shape[2]
                    else:
                        self.get_logger().error(f"❌ Unexpected frame shape: {frame_shape}")
                        return
                    
                    self.get_logger().info(f"🔧 Initializing SLAM with dimensions: {actual_h}x{actual_w}")
                    
                    self.h, self.w = actual_h, actual_w
                    self.mast3r_size = mast3r_size
                    self.manager = mp.Manager()
                    self.main2viz = new_queue(self.manager, not self.enable_visualization)
                    self.viz2main = new_queue(self.manager, not self.enable_visualization)
                    
                    self.keyframes = SharedKeyframes(self.manager, self.h, self.w)
                    self.states = SharedStates(self.manager, self.h, self.w)
                    
                    # Initialize the tracker
                    self.tracker = FrameTracker(self.model, self.keyframes, self.device)
                    self.get_logger().info("🎯 Tracker initialized")
                    
                    # Set up camera intrinsics if available
                    from mast3r_slam.config import config
                    if hasattr(self, 'K') and self.K is not None and config.get('use_calib', False):
                        self.keyframes.set_intrinsics(self.K)
                        self.get_logger().info("✅ Camera intrinsics configured")
                    
                    # 改進 2: 使用 CPU-only 視覺化子程序
                    if self.enable_visualization:
                        self.get_logger().info("🎥 Starting CPU-only visualization process...")
                        self.viz_process = mp.Process(
                            target=_run_visualization_cpu,  # 使用 CPU-only 版本
                            args=(config, self.states, self.keyframes, self.main2viz, self.viz2main),
                        )
                        self.viz_process.start()
                        self.get_logger().info("✅ Visualization started (CPU-only for safety)")
                    else:
                        self.get_logger().info("📺 Visualization disabled")
                    
                    # Start backend process
                    self.get_logger().info("⚙️ Starting backend process...")
                    self.backend_process = mp.Process(
                        target=run_backend, 
                        args=(config, self.model, self.states, self.keyframes, getattr(self, 'K', None))
                    )
                    self.backend_process.start()
                    self.get_logger().info("✅ Backend started")
                    
                    self.slam_initialized = True
                    self.get_logger().info("🚀 SLAM fully initialized!")
                
                # Process the frame
                mode = self.states.get_mode()
                msg = try_get_msg(self.viz2main)
                if msg is not None:
                    self.last_msg = msg
                
                if msg is not None and hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_terminated:
                    self.get_logger().info("🛑 Termination signal received")
                    self.states.set_mode(Mode.TERMINATED)
                    break

                if hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_paused and not self.last_msg.next:
                    self.states.pause()
                    time.sleep(0.01)
                    continue

                if hasattr(self, 'last_msg') and self.last_msg and not self.last_msg.is_paused:
                    self.states.unpause()

                # Create frame
                T_WC = (
                    lietorch.Sim3.Identity(1, device=self.device)
                    if i == 0
                    else self.states.get_frame().T_WC
                )
                
                frame = create_frame(i, img, T_WC, img_size=self.mast3r_size, device=self.device)

                # 改進 4: 使用相機的實際時間戳，而非合成時間
                if hasattr(self, 'dataset'):
                    if len(self.dataset.timestamps) == i:
                        self.dataset.timestamps.append(timestamp)
                    elif len(self.dataset.timestamps) < i:
                        # 填充缺失的時間戳
                        last_ts = self.dataset.timestamps[-1] if self.dataset.timestamps else timestamp
                        gap = i - len(self.dataset.timestamps)
                        # 線性插值填充中間的時間戳
                        for j in range(gap):
                            interp_ts = last_ts + (timestamp - last_ts) * (j + 1) / (gap + 1)
                            self.dataset.timestamps.append(interp_ts)
                        self.dataset.timestamps.append(timestamp)
                    else:
                        # 更新現有的時間戳
                        self.dataset.timestamps[i] = timestamp
                
                # Process based on current mode
                if mode == Mode.INIT:
                    self.get_logger().info("🔧 Initializing SLAM with first frame...")
                    mast3r_stage_start = time.perf_counter()
                    X_init, C_init = mast3r_inference_mono(self.model, frame)
                    self.record_mast3r_stage_timing(time.perf_counter() - mast3r_stage_start)
                    frame.update_pointmap(X_init, C_init)
                    
                    with self.keyframes_lock:
                        self.keyframes.append(frame)
                        
                    with self.states_lock:
                        self.states.queue_global_optimization(len(self.keyframes) - 1)
                        self.states.set_mode(Mode.TRACKING)
                        self.states.set_frame(frame)
                    
                    # 發布第一個 keyframe 的點雲與姿態
                    self.publish_keyframe_pointcloud()
                    self.publish_full_map_pointcloud()
                        
                    self.get_logger().info("✅ SLAM initialized - tracking started")
                    i += 1
                    self.processed_count += 1
                    continue

                add_new_kf = False
                if mode == Mode.TRACKING:
                    mast3r_stage_start = time.perf_counter()
                    add_new_kf, match_info, try_reloc = self.tracker.track(frame)
                    self.record_mast3r_stage_timing(time.perf_counter() - mast3r_stage_start)
                    if try_reloc:
                        with self.states_lock:
                            self.states.set_mode(Mode.RELOC)
                        self.get_logger().info("🔄 Attempting relocalization...")
                    with self.states_lock:
                        self.states.set_frame(frame)

                elif mode == Mode.RELOC:
                    mast3r_stage_start = time.perf_counter()
                    X, C = mast3r_inference_mono(self.model, frame)
                    self.record_mast3r_stage_timing(time.perf_counter() - mast3r_stage_start)
                    frame.update_pointmap(X, C)
                    with self.states_lock:
                        self.states.set_frame(frame)
                        self.states.queue_reloc()

                if add_new_kf:
                    if self.keyframes is not None:
                        with self.keyframes_lock:
                            self.keyframes.append(frame)
                        with self.states_lock:
                            self.states.queue_global_optimization(len(self.keyframes) - 1)
                        self.get_logger().info(f"📍 New keyframe added (total: {len(self.keyframes)})")
                        
                        # 發布最新的 keyframe 點雲到 Unity
                        self.publish_keyframe_pointcloud()
                        
                # 每次 Tracking 結束或新 Keyframe 加入後，發布一次最新的姿態更新
                self.publish_full_map_pointcloud()
                
                # Periodic saving (disabled)
                # current_time = time.time()
                # if (i > 0 and self.keyframes is not None and len(self.keyframes) > 0 and 
                #     current_time - self.last_save_time >= self.save_interval):
                #     self.save_reconstruction_periodic()
                #     
                # # Save on new keyframe (disabled)
                # if i > 0 and self.keyframes is not None and len(self.keyframes) > 0:
                #     current_kf_count = len(self.keyframes)
                #     if (current_kf_count > self.last_keyframe_count and 
                #         current_time - self.last_save_time >= 2.0):
                #         self.save_reconstruction_periodic()
                #         self.last_keyframe_count = current_kf_count
                    
                # Log progress
                if i % 30 == 0 and i > 0:
                    FPS = i / (time.time() - fps_timer)
                    kf_count = len(self.keyframes) if self.keyframes is not None else 0
                    buffer_size = len(self.image_buffer)
                    self.get_logger().info(f"📊 SLAM: {FPS:.2f} FPS, {kf_count} keyframes, Mode: {mode.name}, Buffer: {buffer_size}/3")
                
                i += 1
                self.processed_count += 1
                
            except Exception as e:
                import traceback
                self.get_logger().error(f"🚨 Error in SLAM processing: {str(e)}")
                self.get_logger().error(f"📍 Error type: {type(e).__name__}")
                self.get_logger().error(f"🔄 Traceback: {traceback.format_exc()}")
                
                # Check if critical objects are None
                if not hasattr(self, 'keyframes') or self.keyframes is None:
                    self.get_logger().error("❌ self.keyframes is None")
                if not hasattr(self, 'states') or self.states is None:
                    self.get_logger().error("❌ self.states is None")
                if not hasattr(self, 'viz2main') or self.viz2main is None:
                    self.get_logger().error("❌ self.viz2main is None")
                    
                # Continue processing unless it's a critical initialization error
                if not self.slam_initialized:
                    self.get_logger().error("💥 Critical initialization error, stopping processing")
                    break
                else:
                    self.get_logger().warn("⚠️ Non-critical error, continuing processing...")
                    time.sleep(0.1)
                    continue
        
        # Final save
        self.get_logger().info("💾 Saving final reconstruction...")
        self.save_reconstruction_final()
        self.cleanup_processes()
        
    def save_reconstruction_periodic(self):
        """改進 3: 加鎖防止重入，使用毫秒級檔名避免撞名"""
        # 非阻塞嘗試獲取鎖，如果已有其他儲存在進行則跳過
        if not self.save_lock.acquire(blocking=False):
            self.get_logger().debug("🔒 Save already in progress, skipping...")
            return False
            
        try:
            # 保護儲存過程中的 keyframes 讀取
            with self.keyframes_lock:
                if self.keyframes is None:
                    self.get_logger().warning("⚠️ Cannot save: keyframes is None")
                    return False
                    
                num_keyframes = len(self.keyframes)
                if num_keyframes == 0:
                    self.get_logger().info("📊 No keyframes to save yet...")
                    return False
                    
                # 製作 keyframes 的快照避免競爭
                keyframes_snapshot = list(self.keyframes)
            
            # 確保 logs 目錄存在
            import os
            os.makedirs("logs", exist_ok=True)
            
            # 改進 3-2: 使用毫秒級時間戳避免檔名衝突
            ts = time.time()
            filename = f"{self.save_as}_partial_{ts:.3f}.ply"
            
            # 使用簡化的儲存函數，設定合理的置信度閾值
            c_conf_threshold = 0.7
            
            self.get_logger().info(f"💾 Saving {num_keyframes} keyframes to logs/{filename}")
            
            # 直接儲存到 logs/ 目錄
            save_reconstruction("logs", filename, keyframes_snapshot, c_conf_threshold)
            
            # 更新時間戳
            self.last_save_time = time.time()
            self.get_logger().info(f"✅ Successfully saved: logs/{filename}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error saving reconstruction: {str(e)}")
            import traceback
            self.get_logger().error(f"🔄 Traceback: {traceback.format_exc()}")
            return False
        finally:
            # 確保釋放鎖
            self.save_lock.release()
    
    def save_reconstruction_final(self):
        """改進 3: 最終儲存也加鎖保護"""
        # 使用阻塞鎖確保最終儲存完成
        with self.save_lock:
            try:
                if not hasattr(self, 'keyframes') or self.keyframes is None:
                    self.get_logger().info("ℹ️ No SLAM data to save (system not fully initialized)")
                    return
                    
                num_keyframes = len(self.keyframes)
                if num_keyframes == 0:
                    self.get_logger().info("ℹ️ No keyframes to save")
                    return
                    
                # 確保 logs 目錄存在
                import os
                os.makedirs("logs", exist_ok=True)
                
                # 最終檔名可選：保持 _final 或加時間戳
                # Option 1: 固定檔名（會覆蓋）
                filename = f"{self.save_as}_final.ply"
                # Option 2: 加時間戳（不會覆蓋）
                # ts = time.time()
                # filename = f"{self.save_as}_final_{ts:.3f}.ply"
                
                c_conf_threshold = 0.7
                
                self.get_logger().info(f"📊 Saving final reconstruction: {num_keyframes} keyframes to logs/{filename}")
                
                # 直接儲存到 logs/ 目錄
                save_reconstruction("logs", filename, self.keyframes, c_conf_threshold)
                self.get_logger().info(f"✅ Final reconstruction saved: logs/{filename}")
                
            except Exception as e:
                self.get_logger().error(f"❌ Error saving final reconstruction: {str(e)}")
                import traceback
                self.get_logger().error(f"🔄 Traceback: {traceback.format_exc()}")
    
    # def periodic_save_callback(self):
    #     """改進 3: 定時器觸發的週期性儲存 - 加鎖防止重入"""
    #     # 非阻塞嘗試獲取鎖，如果已有儲存在進行則跳過
    #     if not hasattr(self, 'save_lock') or not self.save_lock.acquire(blocking=False):
    #         # 已有儲存在進行，靜默跳過
    #         return
    #         
    #     try:
    #         if (hasattr(self, 'slam_initialized') and self.slam_initialized and 
    #             hasattr(self, 'keyframes') and self.keyframes is not None and 
    #             len(self.keyframes) > 0):
    #             
    #             # 檢查距離上次儲存的時間間隔
    #             time_since_last_save = time.time() - self.last_save_time
    #             if time_since_last_save >= self.save_interval:
    #                 self.get_logger().info(f"⏰ Timer-triggered save: {len(self.keyframes)} keyframes, {time_since_last_save:.1f}s since last save")
    #                 # 注意：save_reconstruction_periodic 內部也會嘗試獲取鎖
    #                 # 但因為我們已經持有鎖，所以需要先釋放
    #                 self.save_lock.release()
    #                 success = self.save_reconstruction_periodic()
    #                 if success:
    #                     self.get_logger().info("✅ Timer-based save completed for Unity streaming")
    #                 else:
    #                     self.get_logger().debug("⚠️ Timer-based save skipped")
    #                 return  # 已經釋放鎖，直接返回
    #             else:
    #                 self.get_logger().debug(f"⏰ Timer check: only {time_since_last_save:.1f}s since last save, skipping")
    #     except Exception as e:
    #         self.get_logger().error(f"❌ Error in periodic save callback: {str(e)}")
    #         import traceback
    #         self.get_logger().error(f"🔄 Traceback: {traceback.format_exc()}")
    #     finally:
    #         # 確保釋放鎖（如果還持有的話）
    #         try:
    #             self.save_lock.release()
    #         except:
    #             pass  # 鎖可能已經被釋放
    
    def record_mast3r_stage_timing(self, elapsed_sec):
        if elapsed_sec <= 0.0:
            return
        fps = 1.0 / elapsed_sec
        if self.mast3r_stage_fps_ema <= 0.0:
            self.mast3r_stage_fps_ema = fps
        else:
            alpha = self.mast3r_stage_fps_alpha
            self.mast3r_stage_fps_ema = alpha * fps + (1.0 - alpha) * self.mast3r_stage_fps_ema

    def print_stats(self):
        """Print enhanced processing statistics"""
        if self.image_count > 0:
            processing_ratio = self.processed_count / self.image_count * 100
            drop_ratio = self.dropped_count / self.image_count * 100
            
            # Safe keyframe count
            try:
                keyframe_count = len(self.keyframes) if (hasattr(self, 'keyframes') and self.keyframes is not None) else 0
            except (TypeError, AttributeError):
                keyframe_count = 0
            
            time_since_last_save = time.time() - self.last_save_time if self.last_save_time > 0 else 0
            
            mast3r_fps = self.mast3r_stage_fps_ema
            rosbridge_diag = ""
            if getattr(self, "use_rosbridge", False):
                with self._rosbridge_queue_lock:
                    rosbridge_queue_len = len(self._rosbridge_image_queue)
                image_age = -1.0
                wall_lag = -1.0
                if self._last_rosbridge_image_stamp is not None:
                    image_age = time.time() - float(self._last_rosbridge_image_stamp)
                if self._last_rosbridge_image_wall_time is not None:
                    wall_lag = time.time() - float(self._last_rosbridge_image_wall_time)
                rosbridge_diag = (
                    f", rosbridge_rx={self._rosbridge_image_received_count}, "
                    f"rosbridge_overwritten={self._rosbridge_image_overwrite_count}, "
                    f"rosbridge_q={rosbridge_queue_len}, image_age={image_age:.2f}s, "
                    f"wall_lag={wall_lag:.2f}s"
                )
            
            self.get_logger().info(
                f"📈 Stats: {self.image_count} images received, "
                f"{self.processed_count} processed ({processing_ratio:.1f}%), "
                f"{self.dropped_count} dropped ({drop_ratio:.1f}%), "
                f"{keyframe_count} keyframes, "
                f"MASt3R FPS: {mast3r_fps:.2f}, "
                f"Last save: {time_since_last_save:.1f}s ago"
                f"{rosbridge_diag}"
            )
    
    def cleanup_processes(self):
        """Clean up background processes with aggressive termination"""
        print("🧹 Cleaning up background processes...")
        
        # Terminate backend process
        try:
            if hasattr(self, 'backend_process') and self.backend_process and self.backend_process.is_alive():
                print("🛑 Terminating backend process...")
                self.backend_process.terminate()
                self.backend_process.join(timeout=3)
                if self.backend_process.is_alive():
                    print("🔨 Force killing backend process...")
                    self.backend_process.kill()
                    self.backend_process.join(timeout=1)
        except Exception as e:
            print(f"Error cleaning backend process: {e}")
            
        # Terminate visualization process
        try:
            if hasattr(self, 'viz_process') and self.viz_process and self.viz_process.is_alive():
                print("🛑 Terminating visualization process...")
                self.viz_process.terminate()
                self.viz_process.join(timeout=3)
                if self.viz_process.is_alive():
                    print("🔨 Force killing visualization process...")
                    self.viz_process.kill()
                    self.viz_process.join(timeout=1)
        except Exception as e:
            print(f"Error cleaning viz process: {e}")
            
        print("✅ Process cleanup completed")
    
    def destroy_node(self):
        """Clean shutdown of the node"""
        global should_exit, _node_instance
        print("🛑 Node destroy_node() called - initiating comprehensive cleanup...")
        
        should_exit = True
        self.is_processing = False
        
        if hasattr(self, 'processing_thread') and self.processing_thread and self.processing_thread.is_alive():
            print("⏳ Waiting for processing thread to finish...")
            self.processing_thread.join(timeout=5)
        
        if getattr(self, 'use_rosbridge', False):
            self.cleanup_rosbridge()

        self.cleanup_processes()
        
        # Clear global reference
        _node_instance = None
        
        super().destroy_node()
        print("✅ Node destruction completed")


def main(args=None):
    """Main function for ROS2 node"""
    global should_exit
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init(args=args)
    
    try:
        node = MASt3RSLAMVisualizationNode()
        
        print("🚀 MASt3R-SLAM with Visualization started!")
        print("📺 Check the visualization window to monitor SLAM progress")
        print("🔴 Press Ctrl+C to stop and save reconstruction")
        print("")
        print("ℹ️ Performance improvements enabled:")
        print("  • Short FIFO buffer (maxlen=3) for continuous frame processing")
        print("  • CPU-only visualization to avoid CUDA memory issues")
        print("  • Automatic frame dropping when processing lags")
        print("")
        
        # Spin until shutdown
        while rclpy.ok() and not should_exit:
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                print(f"Error in ROS2 spin_once: {str(e)}")
                import traceback
                print(f"Traceback: {traceback.format_exc()}")
                # Don't exit on ROS2 errors, just continue
                time.sleep(0.01)
                continue
            
    except KeyboardInterrupt:
        print("Received KeyboardInterrupt")
    except Exception as e:
        print(f"Error in main: {str(e)}")
        import traceback
        print(f"Traceback: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("MASt3R-SLAM shutdown complete")


if __name__ == '__main__':
    # Register signal handlers for clean shutdown
    signal.signal(signal.SIGINT, cleanup_signal_handler)
    signal.signal(signal.SIGTERM, cleanup_signal_handler)
    
    main()
