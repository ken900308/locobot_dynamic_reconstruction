#!/bin/bash
# Launch MASt3R-SLAM Node for LoCoBot (IGBR Double Buffering Version)
set -e

echo "Starting MASt3R-SLAM (IGBR Architecture)"
echo "========================================================"

ENABLE_VIZ=${ENABLE_VIZ:-false}
ROBOT_ID=${ROBOT_ID:-robot1}
MAST3R_CONFIG=${MAST3R_CONFIG:-config/base.yaml}
MAST3R_SAVE_AS=${MAST3R_SAVE_AS:-${ROBOT_ID}_slam}
MAST3R_IMAGE_TOPIC=${MAST3R_IMAGE_TOPIC:-${IMAGE_TOPIC:-/locobot/camera/camera/color/image_raw/compressed}}
MAST3R_CAMERA_INFO_TOPIC=${MAST3R_CAMERA_INFO_TOPIC:-${CAMERA_INFO_TOPIC:-/locobot/camera/camera/color/camera_info}}
MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}
MAST3R_MAX_FPS=${MAST3R_MAX_FPS:-15.0}
MAST3R_USE_COMPRESSED=${MAST3R_USE_COMPRESSED:-auto}
USE_ROSBRIDGE=${USE_ROSBRIDGE:-false}
ROSBRIDGE_HOST=${ROSBRIDGE_HOST:-192.168.0.60}
ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-9090}
ROSBRIDGE_TF_TOPIC=${ROSBRIDGE_TF_TOPIC:-/tf}
ROSBRIDGE_TF_STATIC_TOPIC=${ROSBRIDGE_TF_STATIC_TOPIC:-/locobot/tf_static_relay}
LOCAL_TF_TOPIC=${LOCAL_TF_TOPIC:-/${ROBOT_ID}/tf}
LOCAL_TF_STATIC_TOPIC=${LOCAL_TF_STATIC_TOPIC:-/${ROBOT_ID}/tf_static}
MAST3R_FRAME_POINTCLOUD_TOPIC=${MAST3R_FRAME_POINTCLOUD_TOPIC:-/${ROBOT_ID}/mast3r/frame_pointcloud}
MAST3R_FULLMAP_POINTCLOUD_TOPIC=${MAST3R_FULLMAP_POINTCLOUD_TOPIC:-/${ROBOT_ID}/mast3r/pointcloud_in_map}
USE_CALIB=${USE_CALIB:-false}

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
        --use-calib)
            USE_CALIB=true
            shift
            ;;
        --no-calib)
            USE_CALIB=false
            shift
            ;;
        --rosbridge)
            USE_ROSBRIDGE=true
            shift
            ;;
        --no-rosbridge)
            USE_ROSBRIDGE=false
            shift
            ;;
        --rosbridge-host)
            ROSBRIDGE_HOST="$2"
            shift 2
            ;;
        --rosbridge-port)
            ROSBRIDGE_PORT="$2"
            shift 2
            ;;
        --tf-topic)
            ROSBRIDGE_TF_TOPIC="$2"
            shift 2
            ;;
        --tf-static-topic)
            ROSBRIDGE_TF_STATIC_TOPIC="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--viz | --no-viz] [--use-calib | --no-calib] [--rosbridge | --no-rosbridge]"
            exit 1
            ;;
    esac
done

if [ ! -d "/workspace/thor/MASt3R-SLAM" ]; then
    echo "Error: Must be run inside the mast3r-slam container"
    exit 1
fi

if [ -f /opt/ros/jazzy/setup.bash ]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    set +u
    source /workspace/thor/ros2_ws/install/setup.bash
    set -u
fi

cd /workspace/thor/MASt3R-SLAM
python3 -c "import torch; torch.cuda.empty_cache();" 2>/dev/null || true
export PYTHONPATH="/workspace/thor/MASt3R-SLAM/thirdparty/mast3r:/workspace/thor/MASt3R-SLAM:${PYTHONPATH:-}"
mkdir -p logs

if [ ! -f "mast3r_slam_visual_IGBR.py" ]; then
    echo "Error: mast3r_slam_visual_IGBR.py not found"
    exit 1
fi

if [ "$USE_CALIB" = "true" ]; then
    MAST3R_CONFIG="config/calib.yaml"
else
    MAST3R_CONFIG="config/base.yaml"
fi

echo ""
echo "Configuration:"
echo "  Robot ID: $ROBOT_ID"
echo "  Visualization: $(if [ "$ENABLE_VIZ" = "true" ]; then echo ENABLED; else echo DISABLED; fi)"
echo "  Config: $MAST3R_CONFIG"
echo "  Save as: $MAST3R_SAVE_AS"
echo "  Image topic: $MAST3R_IMAGE_TOPIC"
echo "  CameraInfo topic: $MAST3R_CAMERA_INFO_TOPIC"
echo "  Device: $MAST3R_DEVICE"
echo "  Max FPS: $MAST3R_MAX_FPS"
echo "  Camera calibration: $(if [ "$USE_CALIB" = "true" ]; then echo ENABLED; else echo DISABLED; fi)"
echo "  Robot transport: $(if [ "$USE_ROSBRIDGE" = "true" ]; then echo "rosbridge $ROSBRIDGE_HOST:$ROSBRIDGE_PORT"; else echo "native DDS"; fi)"
echo "  Remote TF topics: $ROSBRIDGE_TF_TOPIC, $ROSBRIDGE_TF_STATIC_TOPIC"
echo "  Local TF topics: $LOCAL_TF_TOPIC, $LOCAL_TF_STATIC_TOPIC"
echo "  PointCloud outputs: $MAST3R_FRAME_POINTCLOUD_TOPIC, $MAST3R_FULLMAP_POINTCLOUD_TOPIC"
echo ""

cleanup() {
    echo ""
    echo "Shutting down MASt3R-SLAM IGBR..."
    pkill -TERM -f mast3r_slam_visual_IGBR 2>/dev/null || true
    sleep 3
    pkill -9 -f mast3r_slam_visual_IGBR 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ROS_ARGS=(
    --ros-args
    -p "config_file:=$MAST3R_CONFIG"
    -p "save_as:=$MAST3R_SAVE_AS"
    -p "image_topic:=$MAST3R_IMAGE_TOPIC"
    -p "camera_info_topic:=$MAST3R_CAMERA_INFO_TOPIC"
    -p "device:=$MAST3R_DEVICE"
    -p "enable_visualization:=$ENABLE_VIZ"
    -p "max_fps:=$MAST3R_MAX_FPS"
    -p "use_rosbridge:=$USE_ROSBRIDGE"
    -p "rosbridge_host:=$ROSBRIDGE_HOST"
    -p "rosbridge_port:=$ROSBRIDGE_PORT"
    -p "rosbridge_tf_topic:=$ROSBRIDGE_TF_TOPIC"
    -p "rosbridge_tf_static_topic:=$ROSBRIDGE_TF_STATIC_TOPIC"
    -p "frame_pointcloud_topic:=$MAST3R_FRAME_POINTCLOUD_TOPIC"
    -p "fullmap_pointcloud_topic:=$MAST3R_FULLMAP_POINTCLOUD_TOPIC"
    -r "/tf:=$LOCAL_TF_TOPIC"
    -r "/tf_static:=$LOCAL_TF_STATIC_TOPIC"
)

if [ "$MAST3R_USE_COMPRESSED" != "auto" ]; then
    ROS_ARGS+=(-p "use_compressed:=$MAST3R_USE_COMPRESSED")
fi

ROBOT_ID="$ROBOT_ID" python3 mast3r_slam_visual_IGBR.py "${ROS_ARGS[@]}"
