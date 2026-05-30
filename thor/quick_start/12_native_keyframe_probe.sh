#!/bin/bash
set -euo pipefail

source_ros_setup() {
    set +u
    source /opt/ros/jazzy/setup.bash
    if [ -f /workspace/thor/ros_ws/install/setup.bash ]; then
        source /workspace/thor/ros_ws/install/setup.bash
    fi
    if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
        source /workspace/thor/ros2_ws/install/setup.bash
    fi
    set -u
}

echo "Starting Multi-Robot Native Keyframe Probe Node..."
source_ros_setup

ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}
METADATA_TOPIC_TEMPLATE=${METADATA_TOPIC_TEMPLATE:-}
if [ -z "$METADATA_TOPIC_TEMPLATE" ]; then
    METADATA_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_metadata'
fi
SUMMARY_TOPIC=${SUMMARY_TOPIC:-/multi_robot/native_keyframe_summaries}
LOAD_NATIVE_CACHE=${LOAD_NATIVE_CACHE:-false}

echo "  ROBOT_IDS: $ROBOT_IDS"
echo "  METADATA_TOPIC_TEMPLATE: $METADATA_TOPIC_TEMPLATE"
echo "  SUMMARY_TOPIC: $SUMMARY_TOPIC"
echo "  LOAD_NATIVE_CACHE: $LOAD_NATIVE_CACHE"

ros2 run stretch3_ros_nodes multi_robot_native_keyframe_node --ros-args \
    -p robot_ids:="$ROBOT_IDS" \
    -p metadata_topic_template:="$METADATA_TOPIC_TEMPLATE" \
    -p summary_topic:="$SUMMARY_TOPIC" \
    -p load_cache:="$LOAD_NATIVE_CACHE"
