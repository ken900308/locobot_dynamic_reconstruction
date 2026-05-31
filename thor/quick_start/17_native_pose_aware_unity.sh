#!/bin/bash
set -euo pipefail
source_ros_setup(){ set +u; source /opt/ros/jazzy/setup.bash; [ -f /workspace/thor/ros_ws/install/setup.bash ] && source /workspace/thor/ros_ws/install/setup.bash; [ -f /workspace/thor/ros2_ws/install/setup.bash ] && source /workspace/thor/ros2_ws/install/setup.bash; set -u; }
echo "Starting Native MASt3R Pose-Aware Unity Publisher Node..."; source_ros_setup
ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}; NATIVE_CACHE_ROOT=${NATIVE_CACHE_ROOT:-/workspace/shared_native_keyframe_cache}
if [ -z "${METADATA_TOPIC_TEMPLATE:-}" ]; then METADATA_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_metadata'; fi
SCAN_CACHE_ON_START=${SCAN_CACHE_ON_START:-true}; SCAN_PERIOD_SEC=${SCAN_PERIOD_SEC:-1.0}
OPTIMIZED_POSE_TOPIC=${OPTIMIZED_POSE_TOPIC:-/multi_robot/native_optimized_keyframe_poses}; KEYFRAME_CLOUD_TOPIC=${KEYFRAME_CLOUD_TOPIC:-/multi_robot/native_unity_keyframe_clouds}; POSE_GRAPH_TOPIC=${POSE_GRAPH_TOPIC:-/multi_robot/native_unity_pose_graph}; SUMMARY_TOPIC=${SUMMARY_TOPIC:-/multi_robot/native_unity_summaries}
MAST3R_SLAM_ROOT=${MAST3R_SLAM_ROOT:-/workspace/thor/MASt3R-SLAM}; MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}
MIN_CONFIDENCE=${MIN_CONFIDENCE:-0.95}; MAX_POINTS_PER_KEYFRAME=${MAX_POINTS_PER_KEYFRAME:-0}; VOXEL_LEAF_SIZE=${VOXEL_LEAF_SIZE:-0.0}; PUBLISH_PERIOD_SEC=${PUBLISH_PERIOD_SEC:-0.2}
CLOUD_PUBLISH_PERIOD_SEC=${CLOUD_PUBLISH_PERIOD_SEC:-0.25}; MAX_CLOUDS_PER_TICK=${MAX_CLOUDS_PER_TICK:-1}
echo "  ROBOT_IDS: $ROBOT_IDS"; echo "  METADATA_TOPIC_TEMPLATE: $METADATA_TOPIC_TEMPLATE"; echo "  NATIVE_CACHE_ROOT: $NATIVE_CACHE_ROOT"; echo "  SCAN_CACHE_ON_START: $SCAN_CACHE_ON_START"
echo "  OPTIMIZED_POSE_TOPIC: $OPTIMIZED_POSE_TOPIC"; echo "  KEYFRAME_CLOUD_TOPIC: $KEYFRAME_CLOUD_TOPIC"; echo "  POSE_GRAPH_TOPIC: $POSE_GRAPH_TOPIC"; echo "  MIN_CONFIDENCE: $MIN_CONFIDENCE"; echo "  MAX_POINTS_PER_KEYFRAME: $MAX_POINTS_PER_KEYFRAME"; echo "  PUBLISH_PERIOD_SEC: $PUBLISH_PERIOD_SEC"
echo "  CLOUD_PUBLISH_PERIOD_SEC: $CLOUD_PUBLISH_PERIOD_SEC"; echo "  MAX_CLOUDS_PER_TICK: $MAX_CLOUDS_PER_TICK"
ros2 run stretch3_ros_nodes multi_robot_native_pose_aware_unity_node --ros-args \
  -p robot_ids:="$ROBOT_IDS" -p metadata_topic_template:="$METADATA_TOPIC_TEMPLATE" -p native_cache_root:="$NATIVE_CACHE_ROOT" -p scan_cache_on_start:="$SCAN_CACHE_ON_START" -p scan_period_sec:="$SCAN_PERIOD_SEC" \
  -p optimized_pose_topic:="$OPTIMIZED_POSE_TOPIC" -p keyframe_cloud_topic:="$KEYFRAME_CLOUD_TOPIC" -p pose_graph_topic:="$POSE_GRAPH_TOPIC" -p summary_topic:="$SUMMARY_TOPIC" \
  -p mast3r_slam_root:="$MAST3R_SLAM_ROOT" -p device:="$MAST3R_DEVICE" \
  -p min_confidence:="$MIN_CONFIDENCE" -p max_points_per_keyframe:="$MAX_POINTS_PER_KEYFRAME" -p voxel_leaf_size:="$VOXEL_LEAF_SIZE" -p publish_period_sec:="$PUBLISH_PERIOD_SEC" \
  -p cloud_publish_period_sec:="$CLOUD_PUBLISH_PERIOD_SEC" -p max_clouds_per_tick:="$MAX_CLOUDS_PER_TICK"
