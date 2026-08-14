#!/usr/bin/env python3
"""
MASt3R-SLAM ROS2 Node with Visualization

This node integrates the simple image receiver with MASt3R-SLAM processing
and enables visualization for monitoring the inference process.
"""

import sys
import os
import time
import signal
import threading
import queue
import numpy as np
import cv2
import torch
import multiprocessing as mp
from pathlib import Path
import argparse
import yaml
import datetime

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

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
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('enable_visualization', False)  # 預設關閉視覺化
        
        # Get parameters
        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.save_as = self.get_parameter('save_as').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Image buffer and threading
        self.image_buffer = queue.Queue(maxsize=10)  # 增加緩衝區避免阻塞，但仍保持即時性
        self.camera_info = None
        self.processing_thread = None
        self.is_processing = False
        
        # Statistics
        self.image_count = 0
        self.last_timestamp = None  # 用於檢查時間戳穩定性
        self.processed_count = 0
        self.start_time = time.time()
        
        # Thread safety locks
        self.keyframes_lock = threading.Lock()
        self.states_lock = threading.Lock()
        self.last_keyframe_count = 0  # 用於節流存檔
        
        # 週期性儲存管理
        self.save_interval = 5.0  # 每 5 秒儲存一次（與 Unity 即時顯示相配合）
        self.last_save_time = 0
        
        # QoS profiles for stable SLAM processing (改善穩定性)
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 改成可靠傳輸，避免掉包
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # 測試更大緩衝深度，觀察穩定性與延遲的平衡
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,  # 改用原始圖像，避免壓縮損失
            self.image_topic,
            self.image_callback,
            image_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            image_qos  # Use same QoS to avoid compatibility issues
        )
        
        # Timer for stats
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        # Timer for periodic saving (與 Unity 同步)
        self.save_timer = self.create_timer(self.save_interval, self.periodic_save_callback)
        
        self.get_logger().info(f"MASt3R-SLAM Node with Visualization initialized")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"Save as: {self.save_as}")
        self.get_logger().info("🎥 Visualization ENABLED - You can monitor the SLAM process")
        
        # Initialize MASt3R-SLAM components
        self.initialize_slam()
        
    def camera_info_callback(self, msg):
        """Store camera intrinsics from ROS CameraInfo message"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info("Camera intrinsics received!")
            K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"  Resolution: {msg.width}x{msg.height}")
            self.get_logger().info(f"  Intrinsics: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
        
    def image_callback(self, msg):
        """Receive raw images from robot - 阻塞式處理，避免掉幀"""
        try:
            if msg is None:
                self.get_logger().warning("Received None message")
                return
                
            if not hasattr(msg, 'data') or msg.data is None:
                self.get_logger().warning("Message has no data")
                return
            
            # Convert raw image to OpenCV format (無壓縮損失)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            if cv_image is None:
                self.get_logger().warning("Failed to convert raw image")
                return
            
            # Rotate 90 degrees clockwise for vertically positioned D435i
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Convert to 0-1 range for MASt3R
            if cv_image.dtype == np.uint8:
                cv_image = cv_image.astype(np.float32) / 255.0
            
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 檢查時間戳穩定性 (避免大幅跳躍)
            if hasattr(self, 'last_timestamp') and self.last_timestamp is not None:
                dt = timestamp - self.last_timestamp
                if dt > 0.5:  # 超過 0.5 秒的跳躍
                    self.get_logger().warning(f"⚠️  Large timestamp jump: {dt:.3f}s")
                elif dt < 0:  # 時間倒退
                    self.get_logger().warning(f"⚠️  Timestamp went backwards: {dt:.3f}s")
            self.last_timestamp = timestamp
            
            # 阻塞式放入 buffer - 確保每一幀都被處理，像 main.py 一樣
            try:
                self.image_buffer.put((timestamp, cv_image), block=True, timeout=1.0)
                self.image_count += 1
                
                if self.image_count == 1:
                    self.get_logger().info(f"🎉 First image received and rotated!")
                    self.get_logger().info(f"  Shape after rotation: {cv_image.shape}")
                    
            except Exception as e:
                self.get_logger().warning(f"Buffer timeout or error: {str(e)}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def initialize_slam(self):
        """Initialize MASt3R-SLAM components with visualization"""
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
                    self.get_logger().warn("⚠️  Camera info timeout - proceeding with default settings")
                    break
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if should_exit:
                return
                
            # Set up camera intrinsics - we'll determine correct dimensions after MASt3R processing
            self.raw_h, self.raw_w = 640, 480  # After rotation: D435i 480x640 -> 640x480
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
            
            # Camera intrinsics will be set up when SLAM is initialized with correct dimensions
            self.get_logger().info("✅ MASt3R model loaded, ready for SLAM initialization")
            
            # Create a dataset-like object for saving (like RealsenseDataset)
            class SimpleDataset:
                def __init__(self):
                    import pathlib
                    import time
                    self.save_results = True
                    self.timestamps = []
                    self.dataset_path = pathlib.Path("stretch3_ros2_slam")  # Used for saving results
                    self.start_time = time.time()
                    
                def get_timestamps_for_keyframes(self, num_keyframes):
                    """Generate timestamps for keyframes based on current time"""
                    # We need to ensure timestamps exist for all frame_ids, not just keyframe count
                    # Find the maximum frame_id among keyframes to generate sufficient timestamps
                    max_frame_id = 0
                    try:
                        if hasattr(self, '_parent_keyframes') and self._parent_keyframes:
                            for i in range(len(self._parent_keyframes)):
                                kf = self._parent_keyframes[i]
                                if hasattr(kf, 'frame_id'):
                                    max_frame_id = max(max_frame_id, kf.frame_id)
                    except Exception as e:
                        # Fallback: use a generous estimate based on typical SLAM frame-to-keyframe ratios
                        max_frame_id = num_keyframes * 100  # Conservative estimate
                    
                    # Ensure we have timestamps for all frame_ids up to max_frame_id
                    needed_timestamps = max(max_frame_id + 10, num_keyframes)  # Add buffer
                    
                    while len(self.timestamps) < needed_timestamps:
                        # Generate missing timestamps with consistent intervals
                        next_idx = len(self.timestamps)
                        self.timestamps.append(self.start_time + next_idx * 0.033)  # ~30 FPS intervals
                    
                    return self.timestamps
                    
            self.dataset = SimpleDataset()
            
            # Initialize default window message
            self.last_msg = WindowMsg()
            self.get_logger().info(f"🔍 Initial WindowMsg: terminated={self.last_msg.is_terminated}, paused={self.last_msg.is_paused}")
            
            self.get_logger().info(f"🔧 Visualization {'enabled' if self.enable_visualization else 'disabled'}")
            
            # Tracker will be initialized after SLAM setup with correct keyframes object
            self.tracker = None
            
            # Processes will be started after SLAM initialization with correct dimensions
            self.viz_process = None
            self.backend_process = None
            
            # Start processing thread - this will initialize SLAM and start processes
            self.is_processing = True
            self.processing_thread = threading.Thread(target=self.process_images)
            self.processing_thread.start()
            self.get_logger().info("🚀 SLAM processing thread started - waiting for first image to initialize SLAM...")
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"Failed to initialize SLAM: {str(e)}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            raise
    
    def setup_camera_intrinsics(self):
        """Set up camera intrinsics from ROS CameraInfo message with rotation adjustment"""
        K_matrix = np.array(self.camera_info.k).reshape(3, 3)
        
        # Adjust intrinsics for 90° clockwise rotation
        original_width = self.camera_info.width
        original_height = self.camera_info.height
        
        # After 90° clockwise rotation: (x,y) -> (y, width-1-x)
        rotated_intrinsics = {
            'fx': K_matrix[1, 1],  # fy becomes fx
            'fy': K_matrix[0, 0],  # fx becomes fy  
            'cx': K_matrix[1, 2],  # cy becomes cx
            'cy': original_width - 1 - K_matrix[0, 2]  # adjusted cy
        }
        
        # Create intrinsics object - skip complex calibration for now
        # Just use the basic rotated intrinsics directly
        intrinsics = None  # Skip complex calibration
        
        self.get_logger().info("📐 Using simplified intrinsics (skipping undistortion)")
        
        # Store for keyframes - check if intrinsics is None
        if intrinsics is not None:
            self.K = torch.from_numpy(intrinsics.K_frame).to(self.device, dtype=torch.float32)
        else:
            # Fall back to basic intrinsics matrix
            K_basic = np.array([
                [rotated_intrinsics['fx'], 0, rotated_intrinsics['cx']],
                [0, rotated_intrinsics['fy'], rotated_intrinsics['cy']],
                [0, 0, 1]
            ], dtype=np.float32)
            self.K = torch.from_numpy(K_basic).to(self.device, dtype=torch.float32)
        
        self.get_logger().info(f"📐 Camera intrinsics adjusted for rotation:")
        self.get_logger().info(f"  Original: {original_width}x{original_height}")
        self.get_logger().info(f"  Rotated: {original_height}x{original_width}")
        self.get_logger().info(f"  Rotated intrinsics: fx={rotated_intrinsics['fx']:.2f}, fy={rotated_intrinsics['fy']:.2f}")
        
    def process_images(self):
        """Main SLAM processing loop with visualization feedback"""
        global should_exit
        
        i = 0
        fps_timer = time.time()
        
        self.get_logger().info("🔄 Starting SLAM processing loop...")
        
        # Main processing loop - mimic original main.py structure
        while True:  # Infinite loop like original
            try:
                # Check for graceful shutdown signal (like original)
                if should_exit:
                    self.get_logger().info("� Graceful shutdown initiated...")
                    break
                
                # Only check SLAM state and visualization messages after SLAM is initialized
                if self.slam_initialized and hasattr(self, 'states') and self.states:
                    mode = self.states.get_mode()
                    msg = try_get_msg(self.viz2main) if hasattr(self, 'viz2main') else None
                    if msg is not None:
                        self.last_msg = msg
                        if i % 100 == 0:  # Log occasionally
                            self.get_logger().info(f"🖥️  Viz msg: terminated={msg.is_terminated}, paused={msg.is_paused}")
                    
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
                
                # Get image from buffer - wait longer, don't give up easily
                try:
                    timestamp, img = self.image_buffer.get(timeout=2.0)
                    
                    # Validate the image data
                    if img is None:
                        self.get_logger().warning("⚠️  Received None image, skipping...")
                        continue 
                        
                    if not hasattr(img, 'shape') or len(img.shape) != 3:
                        self.get_logger().warning(f"⚠️  Invalid image shape: {img.shape if hasattr(img, 'shape') else 'No shape'}, skipping...")
                        continue
                        
                except queue.Empty:
                    # No new images, but don't exit - just continue waiting (like original waits for dataset)
                    time.sleep(0.01)
                    continue
                
                # Get current mode for processing
                mode = self.states.get_mode() if (self.slam_initialized and hasattr(self, 'states') and self.states) else Mode.INIT
                
                if should_exit:
                    break
                
                # Initialize SLAM with correct dimensions from first frame
                if not self.slam_initialized:
                    # 統一使用 MASt3R 的標準尺寸
                    mast3r_size = 512  # MASt3R 標準處理尺寸
                    
                    # Process first image through MASt3R to get actual dimensions
                    temp_frame = create_frame(0, img, np.eye(4), img_size=mast3r_size, device=self.device)
                    frame_shape = temp_frame.img.shape
                    self.get_logger().info(f"🔍 Frame tensor shape: {frame_shape}")
                    
                    # Extract dimensions correctly from tensor shape: [B, C, H, W] or [C, H, W]
                    if len(frame_shape) == 4:  # Batch format: [B, C, H, W]
                        actual_h, actual_w = frame_shape[2], frame_shape[3]  # H, W from [B, C, H, W]
                    elif len(frame_shape) == 3:  # CHW format: [C, H, W]
                        actual_h, actual_w = frame_shape[1], frame_shape[2]  # H, W from [C, H, W]
                    else:
                        self.get_logger().error(f"❌ Unexpected frame shape: {frame_shape}")
                        return
                    
                    self.get_logger().info(f"🔧 Initializing SLAM with aligned dimensions: {actual_h}x{actual_w} (MASt3R size: {mast3r_size})")
                    
                    # 確保 SharedKeyframes 和 create_frame 使用相同的尺寸
                    self.h, self.w = actual_h, actual_w
                    self.mast3r_size = mast3r_size  # 儲存用於後續 create_frame 調用
                    self.manager = mp.Manager()
                    self.main2viz = new_queue(self.manager, False)  # Enable visualization
                    self.viz2main = new_queue(self.manager, False)
                    
                    self.keyframes = SharedKeyframes(self.manager, self.h, self.w)
                    self.states = SharedStates(self.manager, self.h, self.w)
                    
                    # NOW initialize the tracker with the proper keyframes object
                    self.tracker = FrameTracker(self.model, self.keyframes, self.device)
                    self.get_logger().info("🎯 Tracker initialized with SharedKeyframes")
                    
                    # Optionally set up camera intrinsics for keyframes (like original main.py)
                    # Check if we should use calibration (from config file)
                    from mast3r_slam.config import config
                    if hasattr(self, 'K') and self.K is not None and config.get('use_calib', False):
                        self.keyframes.set_intrinsics(self.K)
                        self.get_logger().info("✅ Camera intrinsics configured for actual dimensions")
                    else:
                        self.get_logger().info("🔧 Running without camera intrinsics (like original MASt3R-SLAM)")
                    
                    # Start visualization process only if enabled
                    if self.enable_visualization:
                        self.get_logger().info("🎥 Starting visualization process...")
                        self.viz_process = mp.Process(
                            target=run_visualization,
                            args=(config, self.states, self.keyframes, self.main2viz, self.viz2main),
                        )
                        self.viz_process.start()
                        self.get_logger().info("✅ Visualization started")
                    else:
                        self.get_logger().info("📺 Visualization disabled (enable_visualization=False)")
                    
                    self.get_logger().info("⚙️  Starting backend process...")
                    self.backend_process = mp.Process(
                        target=run_backend, 
                        args=(config, self.model, self.states, self.keyframes, getattr(self, 'K', None))
                    )
                    self.backend_process.start()
                    self.get_logger().info("✅ Backend started")
                    
                    self.slam_initialized = True
                    self.get_logger().info("🚀 SLAM fully initialized with visualization!")
                
                # Now start continuous processing loop
                self.get_logger().info("🔄 Starting continuous SLAM processing...")
                
                # Continue to process the first frame like the original main.py
                mode = self.states.get_mode()
                msg = try_get_msg(self.viz2main)
                if msg is not None:
                    self.last_msg = msg
                    self.get_logger().info(f"🖥️  Viz msg: terminated={msg.is_terminated}, paused={msg.is_paused}")
                
                # Only break on actual termination signal (not initial state)
                if msg is not None and hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_terminated:
                    self.get_logger().info("🛑 Termination signal received, stopping SLAM")
                    self.states.set_mode(Mode.TERMINATED)
                    break

                if hasattr(self, 'last_msg') and self.last_msg and self.last_msg.is_paused and not self.last_msg.next:
                    self.states.pause()
                    time.sleep(0.01)
                    continue

                if hasattr(self, 'last_msg') and self.last_msg and not self.last_msg.is_paused:
                    self.states.unpause()

                # MASt3R handles resizing, no need to manually resize
                
                # Create frame
                T_WC = (
                    lietorch.Sim3.Identity(1, device=self.device)
                    if i == 0
                    else self.states.get_frame().T_WC
                )
                
                # 使用與 SharedKeyframes 一致的尺寸
                frame = create_frame(i, img, T_WC, img_size=self.mast3r_size, device=self.device)

                # 更新 timestamps（仿照 main.py 的 dataset timestamps）
                current_time = time.time()
                if hasattr(self, 'dataset'):
                    # 確保 timestamps 足夠長度
                    while len(self.dataset.timestamps) <= i:
                        self.dataset.timestamps.append(current_time + len(self.dataset.timestamps) * 0.1)
                
                if mode == Mode.INIT:
                    # Initialize via mono inference
                    self.get_logger().info("🔧 Initializing SLAM with first frame...")
                    X_init, C_init = mast3r_inference_mono(self.model, frame)
                    frame.update_pointmap(X_init, C_init)
                    
                    # 保護 keyframes 更新
                    with self.keyframes_lock:
                        self.keyframes.append(frame)
                        
                    with self.states_lock:
                        self.states.queue_global_optimization(len(self.keyframes) - 1)
                        self.states.set_mode(Mode.TRACKING)
                        self.states.set_frame(frame)
                        
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
                        self.get_logger().info(f"🔑 New keyframe added (total: {len(self.keyframes)})")
                    else:
                        self.get_logger().warning("⚠️  Keyframes is None, cannot add new keyframe")
                        
                # 🔄 週期性儲存：基於時間間隔，為 Unity 即時顯示準備
                current_time = time.time()
                if (i > 0 and self.keyframes is not None and len(self.keyframes) > 0 and 
                    current_time - self.last_save_time >= self.save_interval):
                    self.get_logger().info(f"⏰ Time-based save triggered: {current_time - self.last_save_time:.1f}s since last save")
                    self.save_reconstruction_periodic()
                    
                # 🔑 額外儲存：當有新 keyframe 且間隔足夠時
                if i > 0 and self.keyframes is not None and len(self.keyframes) > 0:
                    current_kf_count = len(self.keyframes)
                    if (current_kf_count > self.last_keyframe_count and 
                        current_time - self.last_save_time >= 2.0):  # 新 keyframe 時至少間隔 2 秒
                        self.get_logger().info(f"🔑 Keyframe-based save triggered: {current_kf_count} keyframes")
                        self.save_reconstruction_periodic()
                        self.last_keyframe_count = current_kf_count
                    
                # Log progress with stability metrics
                if i % 30 == 0 and i > 0:
                    FPS = i / (time.time() - fps_timer)
                    kf_count = len(self.keyframes) if self.keyframes is not None else 0
                    buffer_size = self.image_buffer.qsize()
                    self.get_logger().info(f"📊 SLAM: {FPS:.2f} FPS, {kf_count} keyframes, Mode: {mode.name}, Buffer: {buffer_size}/10")
                
                i += 1
                self.processed_count += 1
                
            except Exception as e:
                import traceback
                self.get_logger().error(f"🚨 Error in SLAM processing: {str(e)}")
                self.get_logger().error(f"🔍 Error type: {type(e).__name__}")
                self.get_logger().error(f"📄 Traceback: {traceback.format_exc()}")
                
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
                    self.get_logger().warn("⚠️  Non-critical error, continuing processing...")
                    time.sleep(0.1)
                    continue
        
        # If we get here, the loop exited normally
        self.get_logger().warn(f"🔍 Main processing loop ended normally at iteration {i}")

        # Final save - but first debug why we exited
        self.get_logger().warn(f"🔍 Processing loop exited: should_exit={should_exit}, is_processing={self.is_processing}")
        if hasattr(self, 'states') and self.states:
            mode = self.states.get_mode()
            self.get_logger().warn(f"🔍 Final SLAM mode: {mode}")
        
        self.get_logger().info("💾 Saving final reconstruction...")
        self.save_reconstruction_final()
        self.cleanup_processes()
        
    def save_reconstruction_periodic(self):
        """簡化的週期性儲存 - 直接存到 logs/ 目錄，像 main.py 一樣"""
        try:
            # 保護存檔過程中的 keyframes 讀取
            with self.keyframes_lock:
                if self.keyframes is None:
                    self.get_logger().warning("⚠️  Cannot save: keyframes is None")
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
            
            # 簡化的檔名生成 - 直接存到 logs/ 目錄
            timestamp = datetime.datetime.now().strftime("%H%M%S")
            filename = f"{self.save_as}_partial_{timestamp}.ply"
            
            # 使用簡化的儲存函數，設定合理的置信度閾值
            c_conf_threshold = 0.7  # 合理的置信度閾值
            
            self.get_logger().info(f"💾 Saving {num_keyframes} keyframes to logs/{filename}")
            
            # 直接儲存到 logs/ 目錄（無需複雜的目錄結構）
            save_reconstruction("logs", filename, keyframes_snapshot, c_conf_threshold)
            
            # 更新時間戳
            self.last_save_time = time.time()
            self.get_logger().info(f"✅ Successfully saved: logs/{filename}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Error saving reconstruction: {str(e)}")
            import traceback
            self.get_logger().error(f"📄 Traceback: {traceback.format_exc()}")
            return False
    
    def save_reconstruction_final(self):
        """簡化的最終儲存 - 直接存到 logs/ 目錄"""
        try:
            if not hasattr(self, 'keyframes') or self.keyframes is None:
                self.get_logger().info("ℹ️  No SLAM data to save (system not fully initialized)")
                return
                
            num_keyframes = len(self.keyframes)
            if num_keyframes == 0:
                self.get_logger().info("ℹ️  No keyframes to save")
                return
                
            # 確保 logs 目錄存在
            import os
            os.makedirs("logs", exist_ok=True)
            
            # 簡化的最終檔名
            filename = f"{self.save_as}_final.ply"
            c_conf_threshold = 0.7  # 合理的置信度閾值
            
            self.get_logger().info(f"📊 Saving final reconstruction: {num_keyframes} keyframes to logs/{filename}")
            
            # 直接儲存到 logs/ 目錄
            save_reconstruction("logs", filename, self.keyframes, c_conf_threshold)
            self.get_logger().info(f"✅ Final reconstruction saved: logs/{filename}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error saving final reconstruction: {str(e)}")
            import traceback
            self.get_logger().error(f"📄 Traceback: {traceback.format_exc()}")
    
    def periodic_save_callback(self):
        """定時器觸發的週期性儲存 - 確保 Unity 能獲得最新點雲"""
        try:
            if (hasattr(self, 'slam_initialized') and self.slam_initialized and 
                hasattr(self, 'keyframes') and self.keyframes is not None and 
                len(self.keyframes) > 0):
                
                self.get_logger().info(f"⏰ Timer-triggered save: {len(self.keyframes)} keyframes available")
                success = self.save_reconstruction_periodic()
                if success:
                    self.get_logger().info("✅ Timer-based save completed for Unity streaming")
                else:
                    self.get_logger().info("⚠️  Timer-based save skipped (no new data)")
            else:
                # Silently skip if SLAM not initialized yet
                pass
        except Exception as e:
            self.get_logger().error(f"❌ Error in periodic save callback: {str(e)}")
            import traceback
            self.get_logger().error(f"📄 Traceback: {traceback.format_exc()}")
    
    def print_stats(self):
        """Print processing statistics"""
        if self.image_count > 0:
            processing_ratio = self.processed_count / self.image_count * 100
            # Safe keyframe count - handle None case properly
            try:
                keyframe_count = len(self.keyframes) if (hasattr(self, 'keyframes') and self.keyframes is not None) else 0
            except (TypeError, AttributeError):
                keyframe_count = 0
            
            time_since_last_save = time.time() - self.last_save_time if self.last_save_time > 0 else 0
            self.get_logger().info(f"📈 Stats: {self.image_count} images received, "
                                 f"{self.processed_count} processed ({processing_ratio:.1f}%), "
                                 f"{keyframe_count} keyframes, "
                                 f"last save: {time_since_last_save:.1f}s ago")
    
    def cleanup_processes(self):
        """Clean up background processes with aggressive termination"""
        print("🧹 Cleaning up background processes...")
        
        # Terminate backend process
        try:
            if hasattr(self, 'backend_process') and self.backend_process.is_alive():
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
            if hasattr(self, 'viz_process') and self.viz_process.is_alive():
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
        
        if hasattr(self, 'processing_thread') and self.processing_thread.is_alive():
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
