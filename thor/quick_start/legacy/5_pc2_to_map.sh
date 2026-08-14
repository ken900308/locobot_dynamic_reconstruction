#!/bin/bash
set -euo pipefail

source_ros_setup() {
    set +u
    source /opt/ros/jazzy/setup.bash
    if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
        source /workspace/thor/ros2_ws/install/setup.bash
    fi
    set -u
}

echo "Starting PC2 to Map Node..."
source_ros_setup

ROBOT_ID=${ROBOT_ID:-robot1}
WORLD_FRAME=${WORLD_FRAME:-locobot/odom}
MAP_FRAME=${MAP_FRAME:-${ROBOT_ID}/mast3r_map}
INPUT_TOPIC=${INPUT_TOPIC:-/${ROBOT_ID}/mast3r/frame_pointcloud}
OUTPUT_TOPIC=${OUTPUT_TOPIC:-/${ROBOT_ID}/mast3r/pointcloud_in_map}
TF_TOPIC=${TF_TOPIC:-/${ROBOT_ID}/tf}
TF_STATIC_TOPIC=${TF_STATIC_TOPIC:-/${ROBOT_ID}/tf_static}

echo "  ROBOT_ID: $ROBOT_ID"
echo "  WORLD_FRAME: $WORLD_FRAME"
echo "  MAP_FRAME: $MAP_FRAME"
echo "  INPUT_TOPIC: $INPUT_TOPIC"
echo "  OUTPUT_TOPIC: $OUTPUT_TOPIC"
echo "  TF_TOPIC: $TF_TOPIC"
echo "  TF_STATIC_TOPIC: $TF_STATIC_TOPIC"

ros2 run stretch3_ros_nodes pc2_to_map_stretch3 --ros-args \
    -p input_topic:="$INPUT_TOPIC" \
    -p output_topic:="$OUTPUT_TOPIC" \
    -p cloud_frame:="$MAP_FRAME" \
    -p world_frame:="$WORLD_FRAME" \
    -r /tf:="$TF_TOPIC" \
    -r /tf_static:="$TF_STATIC_TOPIC"
