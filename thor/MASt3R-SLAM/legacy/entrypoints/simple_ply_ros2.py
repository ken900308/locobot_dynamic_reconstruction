#!/usr/bin/env python3

"""
Simple PLY File Monitor ROS2 Publisher
======================================

This ROS2 node monitors for new .ply files saved by MASt3R-SLAM
and publishes them as PointCloud2 messages for Unity integration.

Simple polling-based approach without external dependencies.

Usage:
    # Terminal 1: Start MASt3R-SLAM (saves .ply files)
    ./run_python_enhanced.sh --dataset realsense
    
    # Terminal 2: Start PLY file monitor
    python simple_ply_ros2.py
    
    # Terminal 3: Test ROS2 topic
    ros2 topic echo /mast3r/pointcloud
"""

import os
import sys
import time
import glob
import argparse
from pathlib import Path

# Fix Python path
sys.path.insert(0, '/workspace/MASt3R-SLAM/thirdparty/mast3r')
sys.path.insert(0, '/workspace/MASt3R-SLAM')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np


class SimplePLYMonitorROS2Publisher(Node):
    """ROS2 node that monitors and publishes point clouds from .ply files"""
    
    def __init__(self, logs_dir: str = "logs", file_pattern: str = "*.ply", 
                 frame_id: str = "mast3r_map", decimation_factor: float = 0.1, 
                 poll_interval: float = 0.5, max_points: int = 120000, 
                 enable_unity_coords: bool = False):  # Default: send raw coordinates
        super().__init__('simple_ply_monitor_publisher')
        
        # Optimized QoS for real-time Unity streaming
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, 
            depth=1,  # Only keep latest message
            reliability=ReliabilityPolicy.BEST_EFFORT  # Don't wait for ACK
        )
        
        # ROS2 publisher with optimized QoS
        self.publisher = self.create_publisher(
            PointCloud2, 
            '/mast3r/pointcloud', 
            qos  # Use optimized QoS instead of default
        )
        
        # Configuration
        self.logs_dir = logs_dir
        self.file_pattern = file_pattern
        self.frame_id = frame_id
        self.decimation_factor = decimation_factor  # Keep only this fraction of points
        self.max_points = max_points  # Maximum points per message for web performance
        self.poll_interval = poll_interval
        self.publish_rate = 2.0  # Publish at 2Hz max for web performance
        self.enable_unity_coords = enable_unity_coords  # Convert coordinates for Unity
        
        # State
        self.last_processed_file = None
        self.last_processed_time = 0
        self.published_count = 0
        self.total_points_published = 0
        
        self.get_logger().info(f"Simple PLY Monitor ROS2 Publisher initialized:")
        self.get_logger().info(f"  Topic: /mast3r/pointcloud")
        self.get_logger().info(f"  Watch Directory: {self.logs_dir}")
        self.get_logger().info(f"  File Pattern: {self.file_pattern}")
        self.get_logger().info(f"  Frame ID: {self.frame_id}")
        self.get_logger().info(f"  Decimation: {self.decimation_factor}")
        self.get_logger().info(f"  Max Points: {self.max_points}")
        self.get_logger().info(f"  Raw Coordinates: {not self.enable_unity_coords} (Unity handles transformation)")
        self.get_logger().info(f"  Check Interval: {self.poll_interval}s")
        
        # Create watch directory if it doesn't exist
        if not os.path.exists(self.logs_dir):
            os.makedirs(self.logs_dir, exist_ok=True)
            self.get_logger().info(f"Created watch directory: {self.logs_dir}")
        
        # Start monitoring timer
        self.timer = self.create_timer(self.poll_interval, self.check_for_new_files)
        self.get_logger().info(f"🔍 Started monitoring for .ply files")
        
        # Process any existing files immediately
        self.check_for_new_files()
    
    def get_latest_ply_file(self):
        """Get the most recently modified .ply file"""
        try:
            ply_pattern = os.path.join(self.logs_dir, self.file_pattern)
            ply_files = glob.glob(ply_pattern)
            
            if not ply_files:
                return None
            
            # Filter out empty or very small files (< 1KB) that might be incomplete
            valid_files = []
            for f in ply_files:
                try:
                    if os.path.getsize(f) > 1024:  # At least 1KB
                        valid_files.append(f)
                except OSError:
                    continue  # Skip files that can't be accessed
            
            if not valid_files:
                return None
            
            # Sort by modification time, get the latest
            latest_file = max(valid_files, key=os.path.getmtime)
            return latest_file
            
        except Exception as e:
            self.get_logger().error(f"Error finding PLY files: {e}")
            return None
    
    def check_for_new_files(self):
        """Check for new or updated .ply files"""
        try:
            latest_file = self.get_latest_ply_file()
            
            if latest_file is None:
                return
            
            # Check if this is a new file or if it's been modified
            file_mtime = os.path.getmtime(latest_file)
            
            # Always process if file is different OR if modification time is newer
            # Also add small tolerance for file system timestamp precision
            if (latest_file != self.last_processed_file or 
                file_mtime > self.last_processed_time + 0.1):  # 0.1s tolerance
                
                self.get_logger().info(f"📄 New/updated PLY file detected: {latest_file}")
                self.get_logger().info(f"   File mtime: {file_mtime}, Last processed: {self.last_processed_time}")
                self.process_ply_file(latest_file)
                self.last_processed_file = latest_file
                self.last_processed_time = file_mtime
                
        except Exception as e:
            self.get_logger().error(f"Error checking for new files: {e}")
    
    def read_ply_file(self, ply_path: str):
        """
        Read point cloud data from a .ply file (supports both ASCII and binary formats)
        
        Args:
            ply_path: Path to the .ply file
            
        Returns:
            Tuple of (points, colors) as numpy arrays, or (None, None) if failed
        """
        try:
            import struct
            points = []
            colors = []
            
            # Read header first to determine format
            with open(ply_path, 'rb') as f:
                header_lines = []
                is_binary = False
                vertex_count = 0
                
                # Read header line by line
                while True:
                    line = f.readline()
                    try:
                        line_str = line.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        break
                    
                    header_lines.append(line_str)
                    
                    if line_str.startswith('format'):
                        if 'binary' in line_str:
                            is_binary = True
                    elif line_str.startswith('element vertex'):
                        vertex_count = int(line_str.split()[-1])
                    elif line_str == 'end_header':
                        header_pos = f.tell()  # Store position after header
                        break
                
                if vertex_count == 0:
                    self.get_logger().warning(f"No vertices found in {ply_path}")
                    return None, None
                
                # Read binary data
                if is_binary:
                    # OPTIMIZED: Vectorized binary reading with numpy
                    # Binary format: each vertex is 3 floats (x,y,z) + 3 uchars (r,g,b) = 15 bytes
                    vertex_dtype = np.dtype([
                        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),  # 3 floats (12 bytes)
                        ('r', 'u1'), ('g', 'u1'), ('b', 'u1')     # 3 unsigned chars (3 bytes)
                    ])
                    
                    try:
                        # Read all vertices at once with numpy (5-10x faster!)
                        vertex_data = np.frombuffer(f.read(vertex_count * vertex_dtype.itemsize), dtype=vertex_dtype)
                        
                        if len(vertex_data) == vertex_count:
                            # Extract coordinates and colors efficiently
                            points = np.column_stack((vertex_data['x'], vertex_data['y'], vertex_data['z'])).astype(np.float32)
                            colors = np.column_stack((vertex_data['r'], vertex_data['g'], vertex_data['b'])).astype(np.uint8)
                        else:
                            self.get_logger().warning(f"Expected {vertex_count} vertices, got {len(vertex_data)}")
                            if len(vertex_data) > 0:
                                points = np.column_stack((vertex_data['x'], vertex_data['y'], vertex_data['z'])).astype(np.float32)
                                colors = np.column_stack((vertex_data['r'], vertex_data['g'], vertex_data['b'])).astype(np.uint8)
                            else:
                                return None, None
                                
                    except Exception as e:
                        self.get_logger().error(f"Vectorized reading failed, falling back to slow method: {e}")
                        # Fallback to old method if vectorized fails
                        f.seek(header_pos)
                        vertex_format = '<fff3B'
                        vertex_size = struct.calcsize(vertex_format)
                        points_list, colors_list = [], []
                        
                        for i in range(vertex_count):
                            data = f.read(vertex_size)
                            if len(data) != vertex_size:
                                break
                            try:
                                x, y, z, r, g, b = struct.unpack(vertex_format, data)
                                points_list.append([x, y, z])
                                colors_list.append([r, g, b])
                            except struct.error:
                                break
                        
                        if points_list:
                            points = np.array(points_list, dtype=np.float32)
                            colors = np.array(colors_list, dtype=np.uint8)
                        else:
                            return None, None
                else:
                    # ASCII format (fallback)
                    f.seek(0)
                    content = f.read().decode('utf-8', errors='ignore')
                    lines = content.split('\n')
                    
                    # Find header end
                    header_end = 0
                    for i, line in enumerate(lines):
                        if line.strip() == 'end_header':
                            header_end = i + 1
                            break
                    
                    # Read vertex data
                    for i in range(header_end, min(header_end + vertex_count, len(lines))):
                        line = lines[i].strip()
                        if not line:
                            continue
                        
                        parts = line.split()
                        if len(parts) >= 6:
                            try:
                                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                                r, g, b = int(float(parts[3])), int(float(parts[4])), int(float(parts[5]))
                                points.append([x, y, z])
                                colors.append([r, g, b])
                            except (ValueError, IndexError):
                                continue
            
            if len(points) == 0:
                self.get_logger().warning(f"No valid points read from {ply_path}")
                return None, None
            
            # Arrays are already created as numpy arrays in optimized section
            self.get_logger().info(f"📊 Read {len(points)} points from {os.path.basename(ply_path)}")
            return points, colors
            
        except Exception as e:
            self.get_logger().error(f"Failed to read PLY file {ply_path}: {e}")
            return None, None
    
    def decimate_pointcloud(self, points: np.ndarray, colors: np.ndarray):
        """Aggressively sample points for web performance"""
        if len(points) == 0:
            return points, colors
        
        # Apply decimation factor first
        if self.decimation_factor < 1.0:
            n_points = len(points)
            n_keep_decimation = int(n_points * self.decimation_factor)
            if n_keep_decimation > 0:
                indices = np.random.choice(n_points, n_keep_decimation, replace=False)
                points = points[indices]
                colors = colors[indices]
        
        # Further limit to max_points for web performance
        if len(points) > self.max_points:
            indices = np.random.choice(len(points), self.max_points, replace=False)
            points = points[indices]
            colors = colors[indices]
            self.get_logger().info(f"🌐 Web optimization: Limited to {self.max_points} points for rosbridge")
        
        return points, colors
    
    def convert_to_unity_coordinates(self, points: np.ndarray) -> np.ndarray:
        """
        Optional coordinate conversion (disabled by default)
        Unity will handle all coordinate transformations and rotations
        """
        if len(points) == 0:
            return points
        
        # Note: This transformation is now disabled by default
        # Unity will receive raw MASt3R-SLAM coordinates and handle:
        # - Rotation alignment (arbitrary up-axis from monocular SLAM)
        # - Scale adjustment
        # - User-controlled orientation
        
        unity_points = points.copy()
        unity_points[:, 0] = points[:, 0]   # X stays X (right)
        unity_points[:, 1] = points[:, 1]   # Y stays Y (up)  
        unity_points[:, 2] = -points[:, 2]  # Z = -Z (flip handedness: back→forward)
        
        return unity_points
    
    def create_pointcloud2_msg(self, points: np.ndarray, colors: np.ndarray) -> PointCloud2:
        """Create PointCloud2 message from points and colors"""
        import struct
        
        msg = PointCloud2()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Point cloud dimensions
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        
        # Field definitions
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.point_step = 16  # 4 bytes * 4 fields
        msg.row_step = msg.point_step * msg.width
        
        # OPTIMIZED: Vectorized PointCloud2 packing with numpy
        num_points = len(points)
        if num_points == 0:
            msg.data = b''
            return msg
        
        # Create structured array for efficient packing
        # Each point: X(4) + Y(4) + Z(4) + RGB(4) = 16 bytes
        point_dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),  # XYZ as float32
            ('rgb', '<u4')  # RGB packed as uint32
        ])
        
        # Allocate structured array
        point_data = np.zeros(num_points, dtype=point_dtype)
        
        # Fill coordinates (vectorized)
        point_data['x'] = points[:, 0].astype(np.float32)
        point_data['y'] = points[:, 1].astype(np.float32) 
        point_data['z'] = points[:, 2].astype(np.float32)
        
        # Pack RGB efficiently (vectorized bit operations)
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)
        point_data['rgb'] = (r << 16) | (g << 8) | b
        
        # Convert to bytes in one operation (much faster!)
        msg.data = point_data.tobytes()
        return msg
    
    def process_ply_file(self, ply_path: str):
        """Process a .ply file and publish as PointCloud2"""
        try:
            # Read PLY file
            points, colors = self.read_ply_file(ply_path)
            if points is None or colors is None:
                return
            
            # Apply decimation
            decimated_points, decimated_colors = self.decimate_pointcloud(points, colors)
            
            if len(decimated_points) == 0:
                self.get_logger().warning("No points remaining after decimation")
                return
            
            # Send raw coordinates to Unity (Unity handles all transformations)
            if self.enable_unity_coords:
                # Optional: apply basic coordinate conversion
                decimated_points = self.convert_to_unity_coordinates(decimated_points)
                self.get_logger().debug("Applied coordinate conversion")
            else:
                # Default: send raw MASt3R-SLAM coordinates
                self.get_logger().debug("Sending raw MASt3R coordinates (Unity will transform)")
            
            # Create and publish PointCloud2 message
            pointcloud_msg = self.create_pointcloud2_msg(decimated_points, decimated_colors)
            self.publisher.publish(pointcloud_msg)
            
            # Update statistics
            self.published_count += 1
            self.total_points_published += len(decimated_points)
            
            filename = os.path.basename(ply_path)
            self.get_logger().info(f"📡 Published {filename}: {len(decimated_points)} points "
                                 f"(decimated from {len(points)}) - Total: {self.published_count}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process PLY file {ply_path}: {e}")


def main():
    """Main function with command line argument support"""
    parser = argparse.ArgumentParser(
        description="ROS2 PLY File Monitor and Publisher for Unity Integration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Monitor default logs directory
  python simple_ply_ros2.py
  
  # Monitor specific directory with Unity coordinate conversion
  python simple_ply_ros2.py --logs-dir /path/to/ply/files --unity-coords
  
  # Custom decimation and max points
  python simple_ply_ros2.py --decimation 0.05 --max-points 200000
  
  # Monitor specific PLY file pattern
  python simple_ply_ros2.py --pattern "scene_*.ply"
        """)
    
    parser.add_argument("--logs-dir", "-d", 
                       default="/workspace/MASt3R-SLAM/logs",
                       help="Directory to monitor for PLY files (default: /workspace/MASt3R-SLAM/logs)")
    
    parser.add_argument("--pattern", "-p",
                       default="*.ply", 
                       help="File pattern to match (default: *.ply)")
    
    parser.add_argument("--frame-id", "-f",
                       default="mast3r_map",
                       help="ROS2 frame ID for point cloud (default: mast3r_map)")
    
    parser.add_argument("--decimation", "-dec",
                       type=float, default=0.1,
                       help="Point decimation factor 0.0-1.0 (default: 0.1)")
    
    parser.add_argument("--max-points", "-m",
                       type=int, default=120000,
                       help="Maximum points per cloud (default: 120000)")
    
    parser.add_argument("--poll-interval", "-i",
                       type=float, default=0.5,
                       help="File check interval in seconds (default: 0.5)")

    parser.add_argument("--unity-coords", "-u",
                       action="store_true",
                       help="Enable Unity coordinate conversion (Z-axis flip)")
    
    parser.add_argument("--verbose", "-v",
                       action="store_true", 
                       help="Enable verbose logging")
    
    args = parser.parse_args()
    
    print("🚀 Starting Simple PLY Monitor ROS2 Publisher")
    print("=" * 55)
    print(f"📂 Watch Directory: {args.logs_dir}")
    print(f"🔍 File Pattern: {args.pattern}")
    print(f"📍 Frame ID: {args.frame_id}")
    print(f"🎯 Decimation: {args.decimation}")
    print(f"📊 Max Points: {args.max_points}")
    print(f"⏰ Poll Interval: {args.poll_interval}s")
    print(f"🎮 Unity Coords: {'Enabled' if args.unity_coords else 'Disabled'}")
    print("")
    
    try:
        rclpy.init()
        node = SimplePLYMonitorROS2Publisher(
            logs_dir=args.logs_dir,
            file_pattern=args.pattern,
            frame_id=args.frame_id,
            decimation_factor=args.decimation,
            poll_interval=args.poll_interval,
            max_points=args.max_points,
            enable_unity_coords=args.unity_coords
        )
        
        print("✅ PLY Monitor initialized")
        print("🔗 Publishing on topic: /mast3r/pointcloud") 
        print("🎮 Unity WebSocket: ws://localhost:9090")
        print("⏹️  Press Ctrl+C to stop")
        print("")
        print("💡 Start MASt3R-SLAM to generate .ply files:")
        print("   ./run_python_enhanced.sh --dataset realsense")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Shutdown complete")


if __name__ == "__main__":
    main()
