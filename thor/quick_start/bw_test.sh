#!/bin/bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
set -u
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    set +u
    source /workspace/thor/ros2_ws/install/setup.bash
    set -u
fi

RAW_TOPICS=${RAW_TOPICS:-/camera_left/camera_left/color/image_raw/compressed,/camera_top/camera_top/color/image_raw/compressed,/camera_right/camera_right/color/image_raw/compressed}
FUSION_TOPIC=${FUSION_TOPIC:-/stretch3/camera/camera/color/image_raw/compressed/fusion}
WINDOW_SEC=${WINDOW_SEC:-5.0}
REPORT_PERIOD_SEC=${REPORT_PERIOD_SEC:-1.0}
USE_ROSBRIDGE=${USE_ROSBRIDGE:-true}
ROSBRIDGE_HOST=${ROSBRIDGE_HOST:-192.168.0.60}
ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-9090}
ROSBRIDGE_CONNECT_TIMEOUT_SEC=${ROSBRIDGE_CONNECT_TIMEOUT_SEC:-5.0}
LOG_DIR=${LOG_DIR:-/workspace/thor/quick_start/logs}
LOG_PATH=${LOG_PATH:-$LOG_DIR/bw_test_$(date +%Y%m%d_%H%M%S).txt}
REPUBLISH_FUSION=${REPUBLISH_FUSION:-true}
FUSION_REPUBLISH_TOPIC=${FUSION_REPUBLISH_TOPIC:-/local/fusion/image_raw/compressed}
FUSION_REPUBLISH_QUEUE_SIZE=${FUSION_REPUBLISH_QUEUE_SIZE:-10}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip|--host|--rosbridge-host)
            ROSBRIDGE_HOST="$2"
            shift 2
            ;;
        --port|--rosbridge-port)
            ROSBRIDGE_PORT="$2"
            shift 2
            ;;
        --dds|--no-rosbridge)
            USE_ROSBRIDGE=false
            shift
            ;;
        --rosbridge)
            USE_ROSBRIDGE=true
            shift
            ;;
        --raw-topics)
            RAW_TOPICS="$2"
            shift 2
            ;;
        --fusion-topic)
            FUSION_TOPIC="$2"
            shift 2
            ;;
        --window-sec)
            WINDOW_SEC="$2"
            shift 2
            ;;
        --report-period-sec)
            REPORT_PERIOD_SEC="$2"
            shift 2
            ;;
        --log-path)
            LOG_PATH="$2"
            shift 2
            ;;
        --republish-fusion)
            REPUBLISH_FUSION=true
            shift
            ;;
        --no-republish-fusion)
            REPUBLISH_FUSION=false
            shift
            ;;
        --fusion-republish-topic)
            FUSION_REPUBLISH_TOPIC="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

echo "Starting compressed image bandwidth test..."
echo "  USE_ROSBRIDGE: $USE_ROSBRIDGE"
echo "  ROSBRIDGE_HOST: $ROSBRIDGE_HOST"
echo "  ROSBRIDGE_PORT: $ROSBRIDGE_PORT"
echo "  RAW_TOPICS: $RAW_TOPICS"
echo "  FUSION_TOPIC: $FUSION_TOPIC"
echo "  WINDOW_SEC: $WINDOW_SEC"
echo "  REPORT_PERIOD_SEC: $REPORT_PERIOD_SEC"
echo "  LOG_PATH: $LOG_PATH"
echo "  REPUBLISH_FUSION: $REPUBLISH_FUSION"
echo "  FUSION_REPUBLISH_TOPIC: $FUSION_REPUBLISH_TOPIC"

ros2 run image_bw_test compressed_image_bw_test_node --ros-args \
    -p raw_topics:="$RAW_TOPICS" \
    -p fusion_topic:="$FUSION_TOPIC" \
    -p window_sec:="$WINDOW_SEC" \
    -p report_period_sec:="$REPORT_PERIOD_SEC" \
    -p use_rosbridge:="$USE_ROSBRIDGE" \
    -p rosbridge_host:="$ROSBRIDGE_HOST" \
    -p rosbridge_port:="$ROSBRIDGE_PORT" \
    -p rosbridge_connect_timeout_sec:="$ROSBRIDGE_CONNECT_TIMEOUT_SEC" \
    -p log_path:="$LOG_PATH" \
    -p republish_fusion:="$REPUBLISH_FUSION" \
    -p fusion_republish_topic:="$FUSION_REPUBLISH_TOPIC" \
    -p fusion_republish_queue_size:="$FUSION_REPUBLISH_QUEUE_SIZE"
