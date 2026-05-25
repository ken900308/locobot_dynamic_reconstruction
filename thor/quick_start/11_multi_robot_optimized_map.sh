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

echo "Starting Multi-Robot Optimized Map Node..."
source_ros_setup

ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}
CLOUD_TOPIC_TEMPLATE=${CLOUD_TOPIC_TEMPLATE:-}
if [ -z "$CLOUD_TOPIC_TEMPLATE" ]; then
    CLOUD_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_cloud_local'
fi
OPTIMIZED_POSE_TOPIC=${OPTIMIZED_POSE_TOPIC:-/multi_robot/optimized_keyframe_poses}
OPTIMIZED_KEYFRAME_CLOUD_TOPIC=${OPTIMIZED_KEYFRAME_CLOUD_TOPIC:-/multi_robot/optimized_keyframe_clouds}
OPTIMIZED_KEYFRAME_CLOUD_SUMMARY_TOPIC=${OPTIMIZED_KEYFRAME_CLOUD_SUMMARY_TOPIC:-/multi_robot/optimized_keyframe_cloud_summaries}
OPTIMIZED_MAP_TOPIC=${OPTIMIZED_MAP_TOPIC:-/multi_robot/optimized_map_points}
OPTIMIZED_MAP_SUMMARY_TOPIC=${OPTIMIZED_MAP_SUMMARY_TOPIC:-/multi_robot/optimized_map_summaries}
OUTPUT_FRAME=${OUTPUT_FRAME:-multi_robot_optimized_map}
MAX_POINTS_PER_KEYFRAME=${MAX_POINTS_PER_KEYFRAME:-30000}
MAX_MERGED_POINTS=${MAX_MERGED_POINTS:-300000}
MIN_CLOUD_CONFIDENCE=${MIN_CLOUD_CONFIDENCE:-0.95}
VOXEL_LEAF_SIZE_PER_KEYFRAME=${VOXEL_LEAF_SIZE_PER_KEYFRAME:-0.0}
VOXEL_LEAF_SIZE_MERGED=${VOXEL_LEAF_SIZE_MERGED:-0.0}
PUBLISH_MERGED_PERIOD_SEC=${PUBLISH_MERGED_PERIOD_SEC:-1.0}
PUBLISH_ONLY_ON_REVISION_CHANGE=${PUBLISH_ONLY_ON_REVISION_CHANGE:-true}

echo "  ROBOT_IDS: $ROBOT_IDS"
echo "  CLOUD_TOPIC_TEMPLATE: $CLOUD_TOPIC_TEMPLATE"
echo "  OPTIMIZED_POSE_TOPIC: $OPTIMIZED_POSE_TOPIC"
echo "  OPTIMIZED_KEYFRAME_CLOUD_TOPIC: $OPTIMIZED_KEYFRAME_CLOUD_TOPIC"
echo "  OPTIMIZED_KEYFRAME_CLOUD_SUMMARY_TOPIC: $OPTIMIZED_KEYFRAME_CLOUD_SUMMARY_TOPIC"
echo "  OPTIMIZED_MAP_TOPIC: $OPTIMIZED_MAP_TOPIC"
echo "  OPTIMIZED_MAP_SUMMARY_TOPIC: $OPTIMIZED_MAP_SUMMARY_TOPIC"
echo "  OUTPUT_FRAME: $OUTPUT_FRAME"
echo "  MAX_POINTS_PER_KEYFRAME: $MAX_POINTS_PER_KEYFRAME"
echo "  MAX_MERGED_POINTS: $MAX_MERGED_POINTS"
echo "  MIN_CLOUD_CONFIDENCE: $MIN_CLOUD_CONFIDENCE"
echo "  VOXEL_LEAF_SIZE_PER_KEYFRAME: $VOXEL_LEAF_SIZE_PER_KEYFRAME"
echo "  VOXEL_LEAF_SIZE_MERGED: $VOXEL_LEAF_SIZE_MERGED"
echo "  PUBLISH_MERGED_PERIOD_SEC: $PUBLISH_MERGED_PERIOD_SEC"
echo "  PUBLISH_ONLY_ON_REVISION_CHANGE: $PUBLISH_ONLY_ON_REVISION_CHANGE"

ros2 run stretch3_ros_nodes multi_robot_optimized_map_node --ros-args \
    -p robot_ids:="$ROBOT_IDS" \
    -p cloud_topic_template:="$CLOUD_TOPIC_TEMPLATE" \
    -p optimized_pose_topic:="$OPTIMIZED_POSE_TOPIC" \
    -p optimized_keyframe_cloud_topic:="$OPTIMIZED_KEYFRAME_CLOUD_TOPIC" \
    -p optimized_keyframe_cloud_summary_topic:="$OPTIMIZED_KEYFRAME_CLOUD_SUMMARY_TOPIC" \
    -p optimized_map_topic:="$OPTIMIZED_MAP_TOPIC" \
    -p optimized_map_summary_topic:="$OPTIMIZED_MAP_SUMMARY_TOPIC" \
    -p output_frame:="$OUTPUT_FRAME" \
    -p max_points_per_keyframe:="$MAX_POINTS_PER_KEYFRAME" \
    -p max_merged_points:="$MAX_MERGED_POINTS" \
    -p min_cloud_confidence:="$MIN_CLOUD_CONFIDENCE" \
    -p voxel_leaf_size_per_keyframe:="$VOXEL_LEAF_SIZE_PER_KEYFRAME" \
    -p voxel_leaf_size_merged:="$VOXEL_LEAF_SIZE_MERGED" \
    -p publish_merged_period_sec:="$PUBLISH_MERGED_PERIOD_SEC" \
    -p publish_only_on_revision_change:="$PUBLISH_ONLY_ON_REVISION_CHANGE"
