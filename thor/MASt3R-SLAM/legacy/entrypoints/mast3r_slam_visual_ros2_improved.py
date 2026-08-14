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
from collections import deque  # 改進 1: 使用 deque 作為短 FIFO
import numpy as np
import cv2
import torch
import multiprocessing as mp
from pathlib import Path
import argparse
import yaml
import datetime
import socket
import struct
import json

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from tf2_msgs.msg import TFMessage
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

class UDSPointCloudSender:
    """UDS sender for PointCloud and PoseArray transfer"""

    MAGIC = b"MSR1"
    VERSION = 1

    # Header: magic(4) version(1) msg_type(1) flags(2) width(4) height(4) encoding(4) timestamp_ns(8) payload_len(4)
    HEADER_STRUCT = struct.Struct("<4sBBHIIIQI")

    MSG_POINTCLOUD = 2
    MSG_POSEARRAY = 3
    ENC_XYZRGB_FLOAT32 = 10
    ENC_JSON = 99

    def __init__(self, socket_path, logger=None):
        self.socket_path = socket_path
        self._sock = None
        self._sock_lock = threading.Lock()
        self._logger = logger

    def _log_warn(self, msg):
        if self._logger is not None:
            self._logger.warning(msg)
        else:
            print(msg)

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
                self._log_warn(f"IPC pointcloud connect failed: {e}")
                try:
                    sock.close()
                except Exception:
                    pass
                self._sock = None
                return False

    def send(self, point_count, timestamp_ns, payload: bytes):
        if not self._connect():
            return False

        header = self.HEADER_STRUCT.pack(
            self.MAGIC,
            self.VERSION,
            self.MSG_POINTCLOUD,
            0,
            point_count,
            1,
            self.ENC_XYZRGB_FLOAT32,
            timestamp_ns,
            len(payload),
        )

        with self._sock_lock:
            try:
                self._sock.sendall(header + payload)
                return True
            except Exception as e:
                self._log_warn(f"IPC pointcloud send failed: {e}")
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                return False
                
    def send_posearray(self, timestamp_ns, payload_str: str):
        if not self._connect():
            return False
            
        payload = payload_str.encode('utf-8')
        header = self.HEADER_STRUCT.pack(
            self.MAGIC,
            self.VERSION,
            self.MSG_POSEARRAY,
            0,
            1,
            1,
            self.ENC_JSON,
            timestamp_ns,
            len(payload),
        )

        with self._sock_lock:
            try:
                self._sock.sendall(header + payload)
                return True
            except Exception as e:
                self._log_warn(f"IPC posearray send failed: {e}")
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
                return False

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
        
        # 新增：Rosbridge 參數（用於不穩定 WiFi 環境）
        self.declare_parameter('use_rosbridge', False)
        self.declare_parameter('rosbridge_host', '192.168.0.212')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('rosbridge_use_compressed', True)  # 新增：使用壓縮圖像
        
        # Get parameters
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.save_as = self.get_parameter('save_as').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        self.max_fps = self.get_parameter('max_fps').get_parameter_value().double_value
        
        # 取得 rosbridge 參數
        self.use_rosbridge = self.get_parameter('use_rosbridge').get_parameter_value().bool_value
        self.rosbridge_host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.rosbridge_port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.rosbridge_use_compressed = self.get_parameter('rosbridge_use_compressed').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
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
        
        # QoS profiles for stable SLAM processing
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠傳輸
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # 減少深度，配合短 FIFO
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create subscribers - 根據配置選擇 ROS2 原生或 rosbridge
        if self.use_rosbridge:
            # 使用 rosbridge (TCP WebSocket) 訂閱
            self.get_logger().info(f"🌐 Using rosbridge: {self.rosbridge_host}:{self.rosbridge_port}")
            self.image_sub = None  # 不使用 ROS2 原生訂閱
            self.ros_client = None
            self.rosbridge_listener = None
            self._rosbridge_image_queue = []  # 用於接收圖像的 queue
            self._rosbridge_queue_lock = threading.Lock()
            # 在主線程初始化 rosbridge（非阻塞）
            self.init_rosbridge_in_main_thread()
        else:
            # 使用 ROS2 原生訂閱 (UDP DDS)
            self.get_logger().info(f"📡 Using ROS2 native subscription")
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                image_qos
            )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            image_qos
        )
        
        # Timer for stats
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        # Timer for periodic saving
        self.save_timer = self.create_timer(self.save_interval, self.periodic_save_callback)
        
        # 新增：逐幀點雲發布器 (Frame-by-frame PointCloud2 publisher)
        frame_pc_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,  # 增加 buffer
            reliability=ReliabilityPolicy.RELIABLE  # 使用 RELIABLE 以相容標準 subscriber
        )
        self.frame_pc_publisher = self.create_publisher(
            PointCloud2,
            '/mast3r/frame_pointcloud',
            frame_pc_qos
        )
        self.frame_pointcloud_frame_id = 'mast3r_map'  # 座標系名稱
        self.min_confidence_threshold = 0.95  # 置信度過濾閾值
        self.get_logger().info(f"🎯 Frame PointCloud publisher: /mast3r/frame_pointcloud")
        
        # 新增：IPC Socket 點雲/姿態發布器 (for cross-container comm)
        self.declare_parameter('ipc_pointcloud_socket', '/tmp/ipc_socket/mast3r_pointcloud.sock')
        ipc_socket = self.get_parameter('ipc_pointcloud_socket').get_parameter_value().string_value
        self.ipc_pointcloud_sender = UDSPointCloudSender(ipc_socket, self.get_logger())
        self.get_logger().info(f"🔌 IPC PointCloud/Pose Sender: {ipc_socket}")
        
        # 新增：Keyframe 姿態更新發布器
        pose_update_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pose_update_publisher = self.create_publisher(
            PoseArray,
            '/mast3r/keyframe_pose_updates',
            pose_update_qos
        )
        self.get_logger().info(f"🔄 Keyframe Pose Update publisher: /mast3r/keyframe_pose_updates")

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
        """Normalize TF frame ids to ROS2-friendly format."""
        if frame_id is None:
            return ''
        normalized = str(frame_id).strip()
        if normalized.startswith('/'):
            normalized = normalized.lstrip('/')
        return normalized
    
    # ================== Rosbridge Functions ==================
    
    def init_rosbridge_in_main_thread(self):
        """在主線程初始化 rosbridge 連接（非阻塞）"""
        try:
            import roslibpy
            self._roslibpy = roslibpy
        except ImportError:
            self.get_logger().error("❌ roslibpy not installed! Run: pip install roslibpy")
            return
        
        try:
            self.get_logger().info(f"🔌 Connecting to rosbridge...")
            self.ros_client = roslibpy.Ros(host=self.rosbridge_host, port=self.rosbridge_port)
            self.ros_client.run()  # 非阻塞，啟動背景線程
            
            # 等待連接
            import time as t
            for i in range(50):  # 最多等 5 秒
                if self.ros_client.is_connected:
                    break
                t.sleep(0.1)
            
            if not self.ros_client.is_connected:
                self.get_logger().error("❌ Failed to connect to rosbridge")
                return
                
            self.get_logger().info(f"✅ Connected to rosbridge @ {self.rosbridge_host}:{self.rosbridge_port}")
            
            # 決定訂閱原始還是壓縮圖像
            if self.rosbridge_use_compressed:
                # 壓縮圖像 topic - 檢查是否已經有 /compressed 後綴
                if self.image_topic.endswith('/compressed'):
                    compressed_topic = self.image_topic
                else:
                    compressed_topic = self.image_topic + '/compressed'
                msg_type = 'sensor_msgs/CompressedImage'
                self.get_logger().info(f"🗜️ Using COMPRESSED images: {compressed_topic}")
            else:
                compressed_topic = self.image_topic
                msg_type = 'sensor_msgs/Image'

            
            # 訂閱 - callback 會在 roslibpy 的背景線程被呼叫
            self.rosbridge_listener = roslibpy.Topic(
                self.ros_client,
                compressed_topic,
                msg_type
            )
            
            # 使用 lambda 把 msg 放入 queue
            def on_message(msg):
                with self._rosbridge_queue_lock:
                    self._rosbridge_image_queue.append(msg)
            
            self.rosbridge_listener.subscribe(on_message)
            self.get_logger().info(f"📷 Subscribed to {compressed_topic} via rosbridge")
            
            # ======== TF Forwarding via rosbridge ========
            # 建立本地 TF publisher
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            
            # rosbridge 訂閱 /tf
            self._rosbridge_tf_queue = []
            self._rosbridge_tf_queue_lock = threading.Lock()
            self.rosbridge_tf_listener = roslibpy.Topic(
                self.ros_client,
                '/tf',
                'tf2_msgs/TFMessage'
            )
            
            def on_tf_message(msg):
                with self._rosbridge_tf_queue_lock:
                    self._rosbridge_tf_queue.append(msg)
            
            self.rosbridge_tf_listener.subscribe(on_tf_message)
            self.get_logger().info(f"🌐 Subscribed to /tf via rosbridge")
            
            # rosbridge 訂閱 /tf_static
            self._rosbridge_tf_static_queue = []
            self._rosbridge_tf_static_queue_lock = threading.Lock()
            self.rosbridge_tf_static_listener = roslibpy.Topic(
                self.ros_client,
                '/tf_static',
                'tf2_msgs/TFMessage'
            )
            
            def on_tf_static_message(msg):
                with self._rosbridge_tf_static_queue_lock:
                    self._rosbridge_tf_static_queue.append(msg)
            
            self.rosbridge_tf_static_listener.subscribe(on_tf_static_message)
            self.get_logger().info(f"🌐 Subscribed to /tf_static via rosbridge")
            
            # 建立 timer 來處理 queue 中的圖像
            self.rosbridge_poll_timer = self.create_timer(0.05, self.poll_rosbridge_queue)
            self.get_logger().info(f"🔄 Rosbridge poll timer started (50ms interval)")
            
        except Exception as e:
            self.get_logger().error(f"❌ Rosbridge init failed: {e}")
            import traceback
            traceback.print_exc()
    
    def poll_rosbridge_queue(self):
        """從 rosbridge queue 中取出圖像和 TF 並處理"""
        # 處理圖像
        with self._rosbridge_queue_lock:
            if self._rosbridge_image_queue:
                msg = self._rosbridge_image_queue.pop(0)
                self.process_rosbridge_image(msg)
        
        # 處理 /tf
        with self._rosbridge_tf_queue_lock:
            while self._rosbridge_tf_queue:
                tf_msg = self._rosbridge_tf_queue.pop(0)
                self.process_rosbridge_tf(tf_msg, is_static=False)
        
        # 處理 /tf_static
        with self._rosbridge_tf_static_queue_lock:
            while self._rosbridge_tf_static_queue:
                tf_msg = self._rosbridge_tf_static_queue.pop(0)
                self.process_rosbridge_tf(tf_msg, is_static=True)
    
    def process_rosbridge_tf(self, msg, is_static=False):
        """將 rosbridge TF 訊息轉換並在本地重新發布"""
        try:
            transforms = msg.get('transforms', [])
            ros2_transforms = []
            
            # 使用本地當前時間，避免時鐘不同步導致的 TF_OLD_DATA 問題
            current_time = self.get_clock().now().to_msg()
            
            for tf in transforms:
                t = TransformStamped()
                
                # Header - 使用本地時間
                header = tf.get('header', {})
                if is_static:
                    # Static TF 不需要時間戳
                    t.header.stamp.sec = 0
                    t.header.stamp.nanosec = 0
                else:
                    # Dynamic TF 使用當前時間
                    t.header.stamp = current_time
                parent_frame = self._normalize_frame_id(header.get('frame_id', ''))
                child_frame = self._normalize_frame_id(tf.get('child_frame_id', ''))
                if not parent_frame or not child_frame:
                    continue
                t.header.frame_id = parent_frame

                # Child frame
                t.child_frame_id = child_frame
                
                # Transform
                transform = tf.get('transform', {})
                translation = transform.get('translation', {})
                rotation = transform.get('rotation', {})
                
                t.transform.translation.x = translation.get('x', 0.0)
                t.transform.translation.y = translation.get('y', 0.0)
                t.transform.translation.z = translation.get('z', 0.0)
                
                t.transform.rotation.x = rotation.get('x', 0.0)
                t.transform.rotation.y = rotation.get('y', 0.0)
                t.transform.rotation.z = rotation.get('z', 0.0)
                t.transform.rotation.w = rotation.get('w', 1.0)
                
                ros2_transforms.append(t)
            
            # 發布到本地 ROS2
            if ros2_transforms:
                if is_static:
                    self.static_tf_broadcaster.sendTransform(ros2_transforms)
                else:
                    self.tf_broadcaster.sendTransform(ros2_transforms)
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error processing rosbridge TF: {e}")
    
    def process_rosbridge_image(self, msg):
        """處理從 rosbridge 收到的圖像（支援原始和壓縮格式）"""
        try:
            import base64
            
            # FPS 節流
            current_time = time.time()
            if self.min_frame_interval > 0:
                if current_time - self.last_frame_time < self.min_frame_interval:
                    self.dropped_count += 1
                    return
                self.last_frame_time = current_time
            
            # 判斷是壓縮還是原始圖像
            if self.rosbridge_use_compressed:
                # CompressedImage 格式
                format_str = msg.get('format', 'jpeg')
                data_base64 = msg['data']
                
                # 解碼 base64
                image_data = base64.b64decode(data_base64)
                
                # 用 OpenCV 解碼 JPEG/PNG
                np_arr = np.frombuffer(image_data, dtype=np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if cv_image is None:
                    self.get_logger().warn(f"Failed to decode compressed image (format: {format_str})")
                    return
                
                # OpenCV 預設 BGR，轉成 RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                
            else:
                # 原始 Image 格式
                width = msg['width']
                height = msg['height']
                encoding = msg['encoding']
                data_base64 = msg['data']
                
                # 解碼 base64 圖像資料
                image_data = base64.b64decode(data_base64)
                
                # 根據編碼轉換為 numpy 陣列
                if encoding in ['rgb8', 'bgr8']:
                    np_arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 3)
                    if encoding == 'bgr8':
                        np_arr = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB)
                    cv_image = np_arr
                elif encoding == 'rgba8':
                    np_arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width, 4)
                    cv_image = cv2.cvtColor(np_arr, cv2.COLOR_RGBA2RGB)
                elif encoding == 'mono8':
                    np_arr = np.frombuffer(image_data, dtype=np.uint8).reshape(height, width)
                    cv_image = cv2.cvtColor(np_arr, cv2.COLOR_GRAY2RGB)
                else:
                    self.get_logger().warn(f"Unsupported encoding: {encoding}")
                    return
            
            # Convert to 0-1 range for MASt3R
            cv_image = cv_image.astype(np.float32) / 255.0
            
            # 提取時間戳
            stamp = msg.get('header', {}).get('stamp', {})
            sec = stamp.get('sec', 0) if isinstance(stamp, dict) else 0
            nanosec = stamp.get('nanosec', 0) if isinstance(stamp, dict) else 0
            timestamp = sec + nanosec * 1e-9
            if timestamp == 0:
                timestamp = time.time()  # 如果沒有時間戳，使用當前時間
            
            # 放入 buffer（與原生 ROS2 callback 相同邏輯）
            with self.buffer_lock:
                old_size = len(self.image_buffer)
                self.image_buffer.append((timestamp, cv_image))
                if old_size == 3:
                    self.dropped_count += 1
                    if self.dropped_count % 50 == 0:
                        self.get_logger().info(f"📊 Dropped {self.dropped_count} frames due to processing lag")
            
            self.image_count += 1
            
            if self.image_count == 1:
                self.get_logger().info(f"🎉 First image received via rosbridge!")
                img_format = "compressed" if self.rosbridge_use_compressed else "raw"
                self.get_logger().info(f"  Shape: {cv_image.shape}, Format: {img_format}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing rosbridge image: {str(e)}")
    
    def cleanup_rosbridge(self):
        """清理 rosbridge 連接"""
        if hasattr(self, 'rosbridge_listener') and self.rosbridge_listener:
            try:
                self.rosbridge_listener.unsubscribe()
            except:
                pass
        if hasattr(self, 'ros_client') and self.ros_client:
            try:
                self.ros_client.terminate()
            except:
                pass
    
    # ================== Camera Callbacks ==================
    
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
                self.setup_camera_intrinsics()
            
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
        """Set up camera intrinsics from ROS CameraInfo message (no rotation adjustment for D415)"""
        K_matrix = np.array(self.camera_info.k).reshape(3, 3)
        
        # D415 橫放：直接使用原始內參，不需要旋轉調整
        width = self.camera_info.width
        height = self.camera_info.height
        
        # Create intrinsics matrix (no rotation adjustment needed)
        K_basic = np.array([
            [K_matrix[0, 0], 0, K_matrix[0, 2]],  # fx, 0, cx
            [0, K_matrix[1, 1], K_matrix[1, 2]],  # 0, fy, cy
            [0, 0, 1]
        ], dtype=np.float32)
        self.K = torch.from_numpy(K_basic).to(self.device, dtype=torch.float32)
        
        self.get_logger().info(f"📐 Camera intrinsics (D415 horizontal):")
        self.get_logger().info(f"  Resolution: {width}x{height}")
        self.get_logger().info(f"  Intrinsics: fx={K_matrix[0,0]:.2f}, fy={K_matrix[1,1]:.2f}, cx={K_matrix[0,2]:.2f}, cy={K_matrix[1,2]:.2f}")
        
    def publish_keyframe_pointcloud(self):
        """發布最新 keyframe 的點雲到 /mast3r/frame_pointcloud"""
        self.get_logger().info("📤 publish_keyframe_pointcloud() called")
        try:
            with self.keyframes_lock:
                if self.keyframes is None or len(self.keyframes) == 0:
                    self.get_logger().warn("📤 No keyframes available")
                    return
                # 取得最新的 keyframe
                keyframe = self.keyframes.last_keyframe()
                if keyframe is None:
                    self.get_logger().warn("📤 last_keyframe() returned None")
                    return
                    
                if keyframe.X_canon is None or keyframe.C is None:
                    self.get_logger().warn("📤 X_canon or C is None")
                    return
                
                # 取得點雲和置信度
                X_canon = keyframe.X_canon  # (H*W, 3) 相機座標系的 3D 點（已 fusion）
                C = keyframe.C              # (H*W, 1) 置信度
                T_WC = keyframe.T_WC       # Sim3 從相機到世界的變換（已優化）
            
            self.get_logger().info(f"📤 Got keyframe data: X_canon shape={X_canon.shape}, C shape={C.shape}")
            
            # 過濾低置信度的點
            valid_mask = (C[:, 0] > self.min_confidence_threshold).cpu().numpy()
            
            if valid_mask.sum() == 0:
                self.get_logger().warn("📤 No valid points after confidence filter")
                return
            
            self.get_logger().info(f"📤 Valid points: {valid_mask.sum()}")
            
            # 不再轉換到世界座標，直接使用相機座標系 (Local coordinates)
            X_local = X_canon  # (H*W, 3)
            
            # 轉換到 numpy，只取有效點
            points_np = X_local.detach().cpu().numpy()[valid_mask].astype(np.float32)
            
            # 從 uimg 取得顏色
            # keyframe.uimg: (H, W, 3) 正規化後的 RGB
            if hasattr(keyframe, 'uimg') and keyframe.uimg is not None:
                uimg = keyframe.uimg.numpy() if hasattr(keyframe.uimg, 'numpy') else keyframe.uimg
                h, w = uimg.shape[:2]
                # 創建像素座標對應的顏色
                flat_colors = (uimg.reshape(-1, 3) * 255).astype(np.uint8)
                colors_np = flat_colors[valid_mask]
            else:
                # 預設白色
                colors_np = np.ones((points_np.shape[0], 3), dtype=np.uint8) * 255
            
            if len(points_np) == 0:
                self.get_logger().warn("📤 No points after processing")
                return
            
            # Unity 左手座標轉換：Z = -Z (ROS 右手 → Unity 左手)
            points_np[:, 2] = -points_np[:, 2]
            
            # 創建 PointCloud2 訊息
            msg = PointCloud2()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            # [NEW] 將 Keyframe ID 放入 frame_id 中，讓 Unity 可以識別這批點雲屬於哪個 Keyframe
            msg.header.frame_id = f"kf_{keyframe.frame_id}"
            
            msg.height = 1
            msg.width = len(points_np)
            msg.is_bigendian = False
            msg.is_dense = True
            
            # 欄位定義：X, Y, Z, RGB
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step = msg.point_step * msg.width
            
            # 高效向量化打包
            point_dtype = np.dtype([
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('rgb', '<u4')
            ])
            
            point_data = np.zeros(len(points_np), dtype=point_dtype)
            point_data['x'] = points_np[:, 0]
            point_data['y'] = points_np[:, 1]
            point_data['z'] = points_np[:, 2]
            
            # 打包 RGB
            r = colors_np[:, 0].astype(np.uint32)
            g = colors_np[:, 1].astype(np.uint32)
            b = colors_np[:, 2].astype(np.uint32)
            point_data['rgb'] = (r << 16) | (g << 8) | b
            
            msg.data = point_data.tobytes()
            
            # [NEW] Broadcast TF for this keyframe
            try:
                # Extract Sim3 data: [qx, qy, qz, qw, tx, ty, tz, s]
                sim3_data = T_WC.data[0].cpu().numpy()
                qx, qy, qz, qw = sim3_data[0], sim3_data[1], sim3_data[2], sim3_data[3]
                tx, ty, tz = sim3_data[4], sim3_data[5], sim3_data[6]
                
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = self.frame_pointcloud_frame_id  # mast3r_map
                t.child_frame_id = "mast3r_camera"
                
                t.transform.translation.x = float(tx)
                t.transform.translation.y = float(ty)
                t.transform.translation.z = float(tz)
                
                t.transform.rotation.x = float(qx)
                t.transform.rotation.y = float(qy)
                t.transform.rotation.z = float(qz)
                t.transform.rotation.w = float(qw)
                
                self.tf_broadcaster.sendTransform(t)
            except Exception as e:
                self.get_logger().error(f"❌ Error broadcasting TF: {e}")

            # 發布
            self.frame_pc_publisher.publish(msg)
            
            # [NEW] Also send over IPC
            timestamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            self.ipc_pointcloud_sender.send(len(points_np), timestamp_ns, point_data.tobytes())
            
            self.get_logger().info(f"✅ Published pointcloud: {len(points_np)} points & TF")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing frame pointcloud: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def publish_keyframe_pose_updates(self):
        """
        檢查 dirty keyframes 並發布姿態更新
        
        訊息格式：
        - header.frame_id: "kf_ids:0,1,5|scales:1.0,1.0,1.0"
        - poses[]: 每個 Pose 對應一個 keyframe 的 T_WC
        """
        try:
            with self.keyframes_lock:
                if self.keyframes is None or len(self.keyframes) == 0:
                    return
                
                # 取得所有 dirty keyframes 的索引
                dirty_idx = self.keyframes.get_dirty_idx()
                
                if len(dirty_idx) == 0:
                    return
                
                # 收集 dirty keyframes 的姿態資訊
                kf_ids = []
                scales = []
                poses = []
                
                for idx in dirty_idx:
                    idx = int(idx)
                    if idx >= len(self.keyframes):
                        continue
                    
                    keyframe = self.keyframes[idx]
                    T_WC = keyframe.T_WC  # Sim3: rotation + translation + scale
                    
                    # 取得 Sim3 的 matrix 並提取資訊
                    # Sim3.matrix() 返回 4x4 矩陣，包含 scale
                    mat = T_WC.matrix()[0].cpu().numpy()  # (4, 4)
                    
                    # 提取 scale (從 Sim3 的 data 中)
                    # Sim3 data format: [qx, qy, qz, qw, tx, ty, tz, s] (8 elements)
                    sim3_data = T_WC.data[0].cpu().numpy()  # (8,)
                    scale = float(sim3_data[7]) if len(sim3_data) > 7 else 1.0
                    
                    # 提取 quaternion (qx, qy, qz, qw)
                    qx, qy, qz, qw = sim3_data[0], sim3_data[1], sim3_data[2], sim3_data[3]
                    
                    # 提取 translation (考慮 scale)
                    tx, ty, tz = sim3_data[4], sim3_data[5], sim3_data[6]
                    
                    # ROS 座標 -> Unity 左手座標轉換：Z = -Z
                    tz_unity = -tz
                    
                    # 創建 Pose 訊息
                    pose = Pose()
                    pose.position.x = float(tx)
                    pose.position.y = float(ty)
                    pose.position.z = float(tz_unity)
                    pose.orientation.x = float(qx)
                    pose.orientation.y = float(qy)
                    pose.orientation.z = float(-qz)  # Unity 左手座標
                    pose.orientation.w = float(qw)
                    
                    kf_ids.append(str(keyframe.frame_id))
                    scales.append(f"{scale:.6f}")
                    poses.append(pose)
            
            if len(poses) == 0:
                return
            
            # 創建 PoseArray 訊息
            msg = PoseArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            # 編碼 keyframe IDs 和 scales 到 frame_id
            msg.header.frame_id = f"kf_ids:{','.join(kf_ids)}|scales:{','.join(scales)}"
            msg.poses = poses
            
            # 發布
            self.pose_update_publisher.publish(msg)
            
            # [NEW] Also send PoseArray over IPC
            timestamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
            
            poses_list = []
            for p in poses:
                poses_list.append({
                    "position": {"x": p.position.x, "y": p.position.y, "z": p.position.z},
                    "orientation": {"x": p.orientation.x, "y": p.orientation.y, "z": p.orientation.z, "w": p.orientation.w}
                })
            posearray_payload = json.dumps({
                "frame_id": msg.header.frame_id,
                "poses": poses_list
            })
            self.ipc_pointcloud_sender.send_posearray(timestamp_ns, posearray_payload)
            
            self.get_logger().info(f"🔄 Published pose updates for {len(poses)} keyframes: {kf_ids}")
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"❌ Error publishing pose updates: {str(e)}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        
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
                    X_init, C_init = mast3r_inference_mono(self.model, frame)
                    frame.update_pointmap(X_init, C_init)
                    
                    with self.keyframes_lock:
                        self.keyframes.append(frame)
                        
                    with self.states_lock:
                        self.states.queue_global_optimization(len(self.keyframes) - 1)
                        self.states.set_mode(Mode.TRACKING)
                        self.states.set_frame(frame)
                    
                    # 發布第一個 keyframe 的點雲（有 fusion）
                    self.publish_keyframe_pointcloud()
                        
                    self.get_logger().info("✅ SLAM initialized - tracking started")
                    i += 1
                    self.processed_count += 1
                    continue

                add_new_kf = False
                if mode == Mode.TRACKING:
                    add_new_kf, match_info, try_reloc = self.tracker.track(frame)
                    if try_reloc:
                        with self.states_lock:
                            self.states.set_mode(Mode.RELOC)
                        self.get_logger().info("🔄 Attempting relocalization...")
                    with self.states_lock:
                        self.states.set_frame(frame)

                elif mode == Mode.RELOC:
                    X, C = mast3r_inference_mono(self.model, frame)
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
                        
                        # 發布最新的 keyframe 點雲到 Unity（有 fusion）
                        self.publish_keyframe_pointcloud()
                
                # 檢查並發布 dirty keyframes 的姿態更新
                self.publish_keyframe_pose_updates()
                
                # Periodic saving
                current_time = time.time()
                if (i > 0 and self.keyframes is not None and len(self.keyframes) > 0 and 
                    current_time - self.last_save_time >= self.save_interval):
                    self.save_reconstruction_periodic()
                    
                # Save on new keyframe
                if i > 0 and self.keyframes is not None and len(self.keyframes) > 0:
                    current_kf_count = len(self.keyframes)
                    if (current_kf_count > self.last_keyframe_count and 
                        current_time - self.last_save_time >= 2.0):
                        self.save_reconstruction_periodic()
                        self.last_keyframe_count = current_kf_count
                    
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
    
    def periodic_save_callback(self):
        """改進 3: 定時器觸發的週期性儲存 - 加鎖防止重入"""
        # 非阻塞嘗試獲取鎖，如果已有儲存在進行則跳過
        if not hasattr(self, 'save_lock') or not self.save_lock.acquire(blocking=False):
            # 已有儲存在進行，靜默跳過
            return
            
        try:
            if (hasattr(self, 'slam_initialized') and self.slam_initialized and 
                hasattr(self, 'keyframes') and self.keyframes is not None and 
                len(self.keyframes) > 0):
                
                # 檢查距離上次儲存的時間間隔
                time_since_last_save = time.time() - self.last_save_time
                if time_since_last_save >= self.save_interval:
                    self.get_logger().info(f"⏰ Timer-triggered save: {len(self.keyframes)} keyframes, {time_since_last_save:.1f}s since last save")
                    # 注意：save_reconstruction_periodic 內部也會嘗試獲取鎖
                    # 但因為我們已經持有鎖，所以需要先釋放
                    self.save_lock.release()
                    success = self.save_reconstruction_periodic()
                    if success:
                        self.get_logger().info("✅ Timer-based save completed for Unity streaming")
                    else:
                        self.get_logger().debug("⚠️ Timer-based save skipped")
                    return  # 已經釋放鎖，直接返回
                else:
                    self.get_logger().debug(f"⏰ Timer check: only {time_since_last_save:.1f}s since last save, skipping")
        except Exception as e:
            self.get_logger().error(f"❌ Error in periodic save callback: {str(e)}")
            import traceback
            self.get_logger().error(f"🔄 Traceback: {traceback.format_exc()}")
        finally:
            # 確保釋放鎖（如果還持有的話）
            try:
                self.save_lock.release()
            except:
                pass  # 鎖可能已經被釋放
    
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
            
            # 計算有效 FPS（實際處理速度）
            elapsed_time = time.time() - self.start_time
            effective_fps = self.processed_count / elapsed_time if elapsed_time > 0 else 0
            
            self.get_logger().info(
                f"📈 Stats: {self.image_count} images received, "
                f"{self.processed_count} processed ({processing_ratio:.1f}%), "
                f"{self.dropped_count} dropped ({drop_ratio:.1f}%), "
                f"{keyframe_count} keyframes, "
                f"Effective FPS: {effective_fps:.2f}, "
                f"Last save: {time_since_last_save:.1f}s ago"
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