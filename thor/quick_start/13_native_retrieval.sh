#!/bin/bash
set -euo pipefail
source_ros_setup(){ set +u; source /opt/ros/jazzy/setup.bash; [ -f /workspace/thor/ros_ws/install/setup.bash ] && source /workspace/thor/ros_ws/install/setup.bash; [ -f /workspace/thor/ros2_ws/install/setup.bash ] && source /workspace/thor/ros2_ws/install/setup.bash; set -u; }
echo "Starting Native MASt3R Cross-Robot Retrieval Node..."; source_ros_setup
ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}; METADATA_TOPIC_TEMPLATE=${METADATA_TOPIC_TEMPLATE:-'/{robot_id}/mast3r/keyframe_metadata'}
CANDIDATE_TOPIC=${CANDIDATE_TOPIC:-/multi_robot/native_retrieval_candidates}; SUMMARY_TOPIC=${SUMMARY_TOPIC:-/multi_robot/native_retrieval_summaries}
MAST3R_SLAM_ROOT=${MAST3R_SLAM_ROOT:-/workspace/thor/MASt3R-SLAM}; MAST3R_CONFIG_PATH=${MAST3R_CONFIG_PATH:-config/base.yaml}; MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}
TOP_K=${TOP_K:-3}; MIN_THRESH=${MIN_THRESH:-0.005}; QUERY_K_MULTIPLIER=${QUERY_K_MULTIPLIER:-4}
echo "  ROBOT_IDS: $ROBOT_IDS"; echo "  TOP_K: $TOP_K"; echo "  MIN_THRESH: $MIN_THRESH"; echo "  MAST3R_DEVICE: $MAST3R_DEVICE"
ros2 run stretch3_ros_nodes multi_robot_native_retrieval_node --ros-args   -p robot_ids:="$ROBOT_IDS" -p metadata_topic_template:="$METADATA_TOPIC_TEMPLATE"   -p candidate_topic:="$CANDIDATE_TOPIC" -p summary_topic:="$SUMMARY_TOPIC"   -p mast3r_slam_root:="$MAST3R_SLAM_ROOT" -p mast3r_config_path:="$MAST3R_CONFIG_PATH" -p device:="$MAST3R_DEVICE"   -p top_k:="$TOP_K" -p min_thresh:="$MIN_THRESH" -p query_k_multiplier:="$QUERY_K_MULTIPLIER"
