#!/bin/bash
# Launch MASt3R-SLAM ROS2 Node with Visualization for Stretch3 Robot
# Improved version with better parameter control and safety checks
set -e

echo "🚀 Starting MASt3R-SLAM with Visualization for Stretch3 Robot"
echo "===================================================="

# Parse command line arguments
ENABLE_VIZ=${ENABLE_VIZ:-false}  # Default: disable visualization for stability
MAX_FPS=${MAX_FPS:-15.0}         # Default: 15 FPS max processing rate
SAVE_AS=${SAVE_AS:-stretch3_slam}
IMAGE_TOPIC=${IMAGE_TOPIC:-/camera_top/camera_top/color/image_raw/compressed}
CAMERA_INFO_TOPIC=${CAMERA_INFO_TOPIC:-/camera_top/camera_top/color/camera_info}
DEVICE=${DEVICE:-cuda:0}

# Help function
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --help              Show this help message"
    echo "  -v, --viz               Enable visualization (CPU-only for safety)"
    echo "  --no-viz                Disable visualization (default)"
    echo "  --fps FPS               Max processing FPS (default: 15.0)"
    echo "  --save-as NAME          Save name prefix (default: stretch3_slam)"
    echo "  --device DEVICE         CUDA device (default: cuda:0)"
    echo "  --image-topic TOPIC     Image topic (default: /camera/camera/color/image_raw)"
    echo "  --info-topic TOPIC      Camera info topic (default: /camera/camera/color/camera_info)"
    echo ""
    echo "Environment variables:"
    echo "  ENABLE_VIZ              Set to 'true' to enable visualization"
    echo "  MAX_FPS                 Maximum processing FPS"
    echo "  SAVE_AS                 Save name prefix"
    echo ""
    echo "Examples:"
    echo "  # Run with visualization (safe CPU-only mode)"
    echo "  $0 --viz"
    echo ""
    echo "  # Run headless with custom FPS"
    echo "  $0 --no-viz --fps 12.0"
    echo ""
    echo "  # Run with environment variables"
    echo "  ENABLE_VIZ=true MAX_FPS=10 $0"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--viz)
            ENABLE_VIZ=true
            shift
            ;;
        --no-viz)
            ENABLE_VIZ=false
            shift
            ;;
        --fps)
            MAX_FPS="$2"
            shift 2
            ;;
        --save-as)
            SAVE_AS="$2"
            shift 2
            ;;
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --image-topic)
            IMAGE_TOPIC="$2"
            shift 2
            ;;
        --info-topic)
            CAMERA_INFO_TOPIC="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

# Check if we're in the container
if [ ! -d "/workspace/MASt3R-SLAM" ]; then
    echo "❌ Error: Must be run inside the mast3r-slam container"
    echo "Run: cd workspace/docker && ./run_mast3r.sh"
    exit 1
fi

# Change to MASt3R-SLAM directory
cd /workspace/MASt3R-SLAM

# Setup ROS2 environment  
echo "🔧 Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

# # Clean up any remaining processes and CUDA memory
# echo "🧹 Cleaning up previous processes..."
# pkill -f mast3r_slam_visual_ros2 2>/dev/null || true
# pkill -f python3 2>/dev/null || true
# pkill -f mast3r 2>/dev/null || true
# sleep 1  # Give processes time to terminate

# Clear CUDA cache if available
echo "🔧 Clearing CUDA cache..."
python3 -c "import torch; torch.cuda.empty_cache(); print('CUDA cache cleared')" 2>/dev/null || echo "CUDA cache clear skipped"

# Set ROS domain ID
export ROS_DOMAIN_ID=2

# Setup Python paths for MASt3R
export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"

# Create logs directory
mkdir -p logs
echo "📁 Logs directory ready: $(pwd)/logs"

# Check if the visualization node exists
if [ ! -f "mast3r_slam_visual_ros2.py" ]; then
    echo "❌ Error: mast3r_slam_visual_ros2.py not found"
    echo "Make sure the file is properly mounted in the container"
    exit 1
fi

# Display configuration
echo ""
echo "📋 Configuration:"
echo "  • Visualization: $(if [ "$ENABLE_VIZ" = "true" ]; then echo "ENABLED (CPU-only)"; else echo "DISABLED (headless)"; fi)"
echo "  • Max FPS: $MAX_FPS"
echo "  • Save prefix: $SAVE_AS"
echo "  • Device: $DEVICE"
echo "  • Image topic: $IMAGE_TOPIC"
echo "  • Camera info: $CAMERA_INFO_TOPIC"
echo ""

if [ "$ENABLE_VIZ" = "true" ]; then
    echo "🎥 Starting MASt3R-SLAM with VISUALIZATION enabled..."
    echo "📺 A visualization window should appear showing the SLAM process"
    echo "⚠️  Note: Visualization runs CPU-only to avoid CUDA memory issues"
    # Force Mesa software rendering for GLX inside container
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=swrast
    export LIBGL_DRIVERS_PATH=/usr/lib/aarch64-linux-gnu/dri
else
    echo "🖥️  Starting MASt3R-SLAM in HEADLESS mode (no visualization)..."
    echo "💡 Tip: Use '--viz' flag to enable visualization"
fi

echo ""
echo "🔴 Press Ctrl+C to stop and save final reconstruction"
echo ""

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down MASt3R-SLAM..."
    
    # Send SIGTERM to the Python process
    pkill -TERM -f mast3r_slam_visual_ros2 2>/dev/null || true
    
    # Wait a bit for graceful shutdown
    sleep 3
    
    # Force kill if still running
    pkill -9 -f mast3r_slam_visual_ros2 2>/dev/null || true
    
    echo "✅ Cleanup completed"
    echo ""
    echo "📁 Check logs/ directory for saved reconstructions:"
    # ls -la logs/*.ply 2>/dev/null || echo "   (No PLY files found yet)"
}

# Set up trap for cleanup
trap cleanup EXIT INT TERM

# Launch the visualization node with parameters
python3 mast3r_slam_visual_ros2_improved_backup.py \
    --ros-args \
    -p config_file:=config/base.yaml \
    -p save_as:=$SAVE_AS \
    -p image_topic:=$IMAGE_TOPIC \
    -p camera_info_topic:=$CAMERA_INFO_TOPIC \
    -p device:=$DEVICE \
    -p enable_visualization:=$ENABLE_VIZ \
    -p max_fps:=$MAX_FPS \
    --log-level info

# This line will only be reached if the Python script exits normally
echo ""
echo "✅ MASt3R-SLAM completed"
echo "📁 Saved reconstructions in logs/ directory:"
ls -la logs/*.ply 2>/dev/null || echo "   (No PLY files found)"