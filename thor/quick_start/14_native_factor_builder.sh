#!/bin/bash
set -euo pipefail
source_ros_setup(){ set +u; source /opt/ros/jazzy/setup.bash; [ -f /workspace/thor/ros_ws/install/setup.bash ] && source /workspace/thor/ros_ws/install/setup.bash; [ -f /workspace/thor/ros2_ws/install/setup.bash ] && source /workspace/thor/ros2_ws/install/setup.bash; set -u; }
echo "Starting Native MASt3R Dense Factor Builder Node..."; source_ros_setup
CANDIDATE_TOPIC=${CANDIDATE_TOPIC:-/multi_robot/native_retrieval_candidates}; EDGE_TOPIC=${EDGE_TOPIC:-/multi_robot/native_factor_edges}; SUMMARY_TOPIC=${SUMMARY_TOPIC:-/multi_robot/native_factor_summaries}
EDGE_CACHE_DIR=${EDGE_CACHE_DIR:-/workspace/shared_native_keyframe_cache/backend/edges}; MAST3R_SLAM_ROOT=${MAST3R_SLAM_ROOT:-/workspace/thor/MASt3R-SLAM}; MAST3R_CONFIG_PATH=${MAST3R_CONFIG_PATH:-config/base.yaml}; MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}
Q_CONF=${Q_CONF:-1.5}; MIN_MATCH_FRAC=${MIN_MATCH_FRAC:-0.1}
echo "  Q_CONF: $Q_CONF"; echo "  MIN_MATCH_FRAC: $MIN_MATCH_FRAC"; echo "  EDGE_CACHE_DIR: $EDGE_CACHE_DIR"
ros2 run stretch3_ros_nodes multi_robot_native_factor_builder_node --ros-args   -p candidate_topic:="$CANDIDATE_TOPIC" -p edge_topic:="$EDGE_TOPIC" -p summary_topic:="$SUMMARY_TOPIC" -p edge_cache_dir:="$EDGE_CACHE_DIR"   -p mast3r_slam_root:="$MAST3R_SLAM_ROOT" -p mast3r_config_path:="$MAST3R_CONFIG_PATH" -p device:="$MAST3R_DEVICE"   -p q_conf:="$Q_CONF" -p min_match_frac:="$MIN_MATCH_FRAC"
