#!/bin/bash
set -euo pipefail

#############################
# Robot-facing rosbridge I/O
#############################
ROBOT_ROSBRIDGE_HOST=${ROBOT_ROSBRIDGE_HOST:-192.168.0.213}
ROBOT_ROSBRIDGE_PORT=${ROBOT_ROSBRIDGE_PORT:-9090}
ROBOT_IMAGE_TOPIC=${ROBOT_IMAGE_TOPIC:-/locobot/camera/camera/color/image_raw/compressed}
ROBOT_CAMERA_INFO_TOPIC=${ROBOT_CAMERA_INFO_TOPIC:-/locobot/camera/camera/color/camera_info}
ROBOT_TF_TOPIC=${ROBOT_TF_TOPIC:-/tf}
ROBOT_TF_STATIC_TOPIC=${ROBOT_TF_STATIC_TOPIC:-/locobot/tf_static_relay}

#############################
# Local MASt3R settings
#############################
ROBOT_ID=${ROBOT_ID:-robot1}
MAST3R_CONFIG=${MAST3R_CONFIG:-config/base.yaml}
MAST3R_MAX_FPS=${MAST3R_MAX_FPS:-15.0}
MAST3R_USE_COMPRESSED=${MAST3R_USE_COMPRESSED:-auto}
LOCAL_TF_TOPIC=${LOCAL_TF_TOPIC:-/${ROBOT_ID}/tf}
LOCAL_TF_STATIC_TOPIC=${LOCAL_TF_STATIC_TOPIC:-/${ROBOT_ID}/tf_static}
MAST3R_FRAME_POINTCLOUD_TOPIC=${MAST3R_FRAME_POINTCLOUD_TOPIC:-/${ROBOT_ID}/mast3r/frame_pointcloud}
MAST3R_FULLMAP_POINTCLOUD_TOPIC=${MAST3R_FULLMAP_POINTCLOUD_TOPIC:-/${ROBOT_ID}/mast3r/pointcloud_in_map}
MAST3R_KEYFRAME_METADATA_TOPIC=${MAST3R_KEYFRAME_METADATA_TOPIC:-/${ROBOT_ID}/mast3r/keyframe_metadata}
MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC=${MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC:-/${ROBOT_ID}/mast3r/keyframe_cloud_local}
MAST3R_KEYFRAME_IMAGE_TOPIC=${MAST3R_KEYFRAME_IMAGE_TOPIC:-/${ROBOT_ID}/mast3r/keyframe_image}
MAST3R_NATIVE_KEYFRAME_CACHE_DIR=${MAST3R_NATIVE_KEYFRAME_CACHE_DIR:-/workspace/shared_native_keyframe_cache/${ROBOT_ID}}
USE_ROSBRIDGE=${USE_ROSBRIDGE:-true}
USE_CALIB=${USE_CALIB:-false}

FORWARD_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip|--host|--rosbridge-host)
            ROBOT_ROSBRIDGE_HOST="$2"
            shift 2
            ;;
        --port|--rosbridge-port)
            ROBOT_ROSBRIDGE_PORT="$2"
            shift 2
            ;;
        --image-topic)
            ROBOT_IMAGE_TOPIC="$2"
            shift 2
            ;;
        --camera-info-topic|--info-topic)
            ROBOT_CAMERA_INFO_TOPIC="$2"
            shift 2
            ;;
        --tf-topic)
            ROBOT_TF_TOPIC="$2"
            shift 2
            ;;
        --tf-static-topic)
            ROBOT_TF_STATIC_TOPIC="$2"
            shift 2
            ;;
        --use-calib)
            USE_CALIB=true
            FORWARD_ARGS+=(--use-calib)
            shift
            ;;
        --no-calib)
            USE_CALIB=false
            FORWARD_ARGS+=(--no-calib)
            shift
            ;;
        --dds|--no-rosbridge)
            USE_ROSBRIDGE=false
            FORWARD_ARGS+=(--no-rosbridge)
            shift
            ;;
        --rosbridge)
            USE_ROSBRIDGE=true
            FORWARD_ARGS+=(--rosbridge)
            shift
            ;;
        --*)
            FORWARD_ARGS+=("$1")
            shift
            ;;
        *)
            if [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ || "$1" == localhost || "$1" == *.* ]]; then
                ROBOT_ROSBRIDGE_HOST="$1"
            else
                FORWARD_ARGS+=("$1")
            fi
            shift
            ;;
    esac
done

export ROBOT_ID
export MAST3R_SAVE_AS=${MAST3R_SAVE_AS:-${ROBOT_ID}_slam}
export MAST3R_CONFIG
export MAST3R_IMAGE_TOPIC="$ROBOT_IMAGE_TOPIC"
export MAST3R_CAMERA_INFO_TOPIC="$ROBOT_CAMERA_INFO_TOPIC"
export MAST3R_MAX_FPS
export MAST3R_USE_COMPRESSED
export USE_ROSBRIDGE
export USE_CALIB
export ROSBRIDGE_HOST="$ROBOT_ROSBRIDGE_HOST"
export ROSBRIDGE_PORT="$ROBOT_ROSBRIDGE_PORT"
export ROSBRIDGE_TF_TOPIC="$ROBOT_TF_TOPIC"
export ROSBRIDGE_TF_STATIC_TOPIC="$ROBOT_TF_STATIC_TOPIC"
export LOCAL_TF_TOPIC
export LOCAL_TF_STATIC_TOPIC
export MAST3R_FRAME_POINTCLOUD_TOPIC
export MAST3R_FULLMAP_POINTCLOUD_TOPIC
export MAST3R_KEYFRAME_METADATA_TOPIC
export MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC
export MAST3R_KEYFRAME_IMAGE_TOPIC
export MAST3R_NATIVE_KEYFRAME_CACHE_DIR

echo "Starting MASt3R-SLAM as a direct ROS node..."
echo "  ROBOT_ID: $ROBOT_ID"
echo "  MAST3R_CONFIG: $MAST3R_CONFIG"
echo "  MAST3R_SAVE_AS: $MAST3R_SAVE_AS"
echo "  ROBOT_IMAGE_TOPIC: $ROBOT_IMAGE_TOPIC"
echo "  ROBOT_CAMERA_INFO_TOPIC: $ROBOT_CAMERA_INFO_TOPIC"
echo "  ROBOT_TF_TOPIC: $ROBOT_TF_TOPIC"
echo "  ROBOT_TF_STATIC_TOPIC: $ROBOT_TF_STATIC_TOPIC"
echo "  LOCAL_TF_TOPIC: $LOCAL_TF_TOPIC"
echo "  LOCAL_TF_STATIC_TOPIC: $LOCAL_TF_STATIC_TOPIC"
echo "  MAST3R_FRAME_POINTCLOUD_TOPIC: $MAST3R_FRAME_POINTCLOUD_TOPIC"
echo "  MAST3R_FULLMAP_POINTCLOUD_TOPIC: $MAST3R_FULLMAP_POINTCLOUD_TOPIC"
echo "  MAST3R_KEYFRAME_METADATA_TOPIC: $MAST3R_KEYFRAME_METADATA_TOPIC"
echo "  MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC: $MAST3R_KEYFRAME_LOCAL_CLOUD_TOPIC"
echo "  MAST3R_KEYFRAME_IMAGE_TOPIC: $MAST3R_KEYFRAME_IMAGE_TOPIC"
echo "  MAST3R_NATIVE_KEYFRAME_CACHE_DIR: $MAST3R_NATIVE_KEYFRAME_CACHE_DIR"
echo "  USE_ROSBRIDGE: $USE_ROSBRIDGE"
echo "  ROSBRIDGE: $ROSBRIDGE_HOST:$ROSBRIDGE_PORT"
echo "  USE_CALIB: $USE_CALIB"
echo "  Local outputs: $MAST3R_FRAME_POINTCLOUD_TOPIC and $MAST3R_FULLMAP_POINTCLOUD_TOPIC"

exec /workspace/thor/MASt3R-SLAM/launch_mast3r_visual_ros2_igbr.sh "${FORWARD_ARGS[@]}"
