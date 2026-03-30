#!/bin/bash
# Launch MASt3R-SLAM Node for Stretch3 Robot (IGBR Double Buffering Version)
set -e

echo "🚀 Starting MASt3R-SLAM (IGBR Architecture)"
echo "========================================================"

# Default arguments
ENABLE_VIZ=${ENABLE_VIZ:-false}
IPC_SOCKET=${IPC_SOCKET:-/tmp/ipc_socket/mast3r_image.sock}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--viz)
            ENABLE_VIZ=true
            shift
            ;;
        --no-viz)
            ENABLE_VIZ=false
            shift
            ;;
        *)
            echo "❌ Unknown option: $1"
            echo "Usage: $0 [--viz | --no-viz]"
            exit 1
            ;;
    esac
done

# Check if we're in the container
if [ ! -d "/workspace/thor/MASt3R-SLAM" ]; then
    echo "❌ Error: Must be run inside the mast3r-slam container"
    echo "Run: cd workspace/docker && ./run_mast3r.sh"
    exit 1
fi

# Change to MASt3R-SLAM directory
cd /workspace/thor/MASt3R-SLAM

# Clear CUDA cache if available
echo "🔧 Clearing CUDA cache..."
python3 -c "import torch; torch.cuda.empty_cache();" 2>/dev/null || true

# Setup Python paths for MASt3R
export PYTHONPATH="/workspace/thor/MASt3R-SLAM/thirdparty/mast3r:/workspace/thor/MASt3R-SLAM:${PYTHONPATH:-}"

# Create logs directory
mkdir -p logs
echo "📁 Logs directory ready: $(pwd)/logs"

# Check if the visualization node exists
if [ ! -f "mast3r_slam_visual_IGBR.py" ]; then
    echo "❌ Error: mast3r_slam_visual_IGBR.py not found"
    echo "Make sure the file is properly mounted in the container"
    exit 1
fi

# Display configuration
echo ""
echo "📋 Configuration:"
echo "  • Visualization: $(if [ "$ENABLE_VIZ" = "true" ]; then echo "ENABLED"; else echo "DISABLED (headless)"; fi)"
echo "  • IPC socket: $IPC_SOCKET"
echo ""

if [ "$ENABLE_VIZ" = "true" ]; then
    echo "🎥 Starting MASt3R-SLAM with VISUALIZATION enabled..."
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
    echo "🛑 Shutting down MASt3R-SLAM IGBR..."
    
    # Send SIGTERM to the Python process
    pkill -TERM -f mast3r_slam_visual_IGBR 2>/dev/null || true
    
    # Wait a bit for graceful shutdown
    sleep 3
    
    # Force kill if still running
    pkill -9 -f mast3r_slam_visual_IGBR 2>/dev/null || true
    
    echo "✅ Cleanup completed"
    echo ""
    echo "📁 Check logs/ directory for saved reconstructions:"
}

# Set up trap for cleanup
trap cleanup EXIT INT TERM

# Launch the node
ENABLE_VIZ="$ENABLE_VIZ" \
IPC_SOCKET="$IPC_SOCKET" \
python3 mast3r_slam_visual_IGBR.py

# This line will only be reached if the Python script exits normally
echo ""
echo "✅ MASt3R-SLAM (IGBR) completed"
echo "📁 Saved reconstructions in logs/ directory:"
ls -la logs/*.ply 2>/dev/null || echo "   (No PLY files found)"
