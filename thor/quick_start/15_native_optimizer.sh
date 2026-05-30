#!/bin/bash
set -euo pipefail
source_ros_setup(){ set +u; source /opt/ros/jazzy/setup.bash; [ -f /workspace/thor/ros_ws/install/setup.bash ] && source /workspace/thor/ros_ws/install/setup.bash; [ -f /workspace/thor/ros2_ws/install/setup.bash ] && source /workspace/thor/ros2_ws/install/setup.bash; set -u; }
echo "Starting Native MASt3R Keyframe-Level Optimizer Node..."; source_ros_setup
EDGE_TOPIC=${EDGE_TOPIC:-/multi_robot/native_factor_edges}; OPTIMIZED_POSE_TOPIC=${OPTIMIZED_POSE_TOPIC:-/multi_robot/native_optimized_keyframe_poses}; COMPAT_OPTIMIZED_POSE_TOPIC=${COMPAT_OPTIMIZED_POSE_TOPIC:-/multi_robot/optimized_keyframe_poses}; SUMMARY_TOPIC=${SUMMARY_TOPIC:-/multi_robot/native_optimizer_summaries}
ANCHOR_ROBOT=${ANCHOR_ROBOT:-robot1}; MAST3R_SLAM_ROOT=${MAST3R_SLAM_ROOT:-/workspace/thor/MASt3R-SLAM}; MAST3R_CONFIG_PATH=${MAST3R_CONFIG_PATH:-config/base.yaml}; MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}; USE_CALIB=${USE_CALIB:-false}
echo "  ANCHOR_ROBOT: $ANCHOR_ROBOT"; echo "  EDGE_TOPIC: $EDGE_TOPIC"; echo "  MAST3R_DEVICE: $MAST3R_DEVICE"; echo "  USE_CALIB: $USE_CALIB"
ros2 run stretch3_ros_nodes multi_robot_native_optimizer_node --ros-args   -p edge_topic:="$EDGE_TOPIC" -p optimized_pose_topic:="$OPTIMIZED_POSE_TOPIC" -p compat_optimized_pose_topic:="$COMPAT_OPTIMIZED_POSE_TOPIC" -p summary_topic:="$SUMMARY_TOPIC"   -p anchor_robot:="$ANCHOR_ROBOT" -p mast3r_slam_root:="$MAST3R_SLAM_ROOT" -p mast3r_config_path:="$MAST3R_CONFIG_PATH" -p device:="$MAST3R_DEVICE" -p use_calib:="$USE_CALIB"
