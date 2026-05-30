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

echo "Starting Multi-Robot Geometric Verifier Node..."
source_ros_setup

ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}
CLOUD_TOPIC_TEMPLATE=${CLOUD_TOPIC_TEMPLATE:-}
if [ -z "$CLOUD_TOPIC_TEMPLATE" ]; then
    CLOUD_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_cloud_local'
fi
IMAGE_TOPIC_TEMPLATE=${IMAGE_TOPIC_TEMPLATE:-}
if [ -z "$IMAGE_TOPIC_TEMPLATE" ]; then
    IMAGE_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_image'
fi
VERIFICATION_JOB_TOPIC=${VERIFICATION_JOB_TOPIC:-/multi_robot/geometric_verification_jobs}
VERIFICATION_RESULT_TOPIC=${VERIFICATION_RESULT_TOPIC:-/multi_robot/geometric_verification_results}
VERIFICATION_SUMMARY_TOPIC=${VERIFICATION_SUMMARY_TOPIC:-/multi_robot/geometric_verification_summaries}
POSE_CONSTRAINT_TOPIC=${POSE_CONSTRAINT_TOPIC:-/multi_robot/pose_constraints}
MIN_POINTS=${MIN_POINTS:-2000}
MAX_SUMMARY_POINTS=${MAX_SUMMARY_POINTS:-20000}
MAX_FEATURES=${MAX_FEATURES:-1500}
MAX_FEATURE_MATCHES=${MAX_FEATURE_MATCHES:-300}
FEATURE_RATIO=${FEATURE_RATIO:-0.75}
MAX_CORRESPONDENCE_PX=${MAX_CORRESPONDENCE_PX:-6}
MIN_3D_CORRESPONDENCES=${MIN_3D_CORRESPONDENCES:-18}
MIN_INLIERS=${MIN_INLIERS:-12}
RANSAC_ITERATIONS=${RANSAC_ITERATIONS:-160}
RANSAC_INLIER_THRESHOLD_M=${RANSAC_INLIER_THRESHOLD_M:-0.20}
MAX_RMSE_M=${MAX_RMSE_M:-0.18}
MIN_INLIER_RATIO=${MIN_INLIER_RATIO:-0.55}
MIN_CLOUD_CONFIDENCE=${MIN_CLOUD_CONFIDENCE:-0.0}
VERIFIER_BACKEND=${VERIFIER_BACKEND:-mast3r_symmetric}
MAST3R_SLAM_ROOT=${MAST3R_SLAM_ROOT:-/workspace/thor/MASt3R-SLAM}
MAST3R_MODEL_PATH=${MAST3R_MODEL_PATH:-}
MAST3R_CONFIG_PATH=${MAST3R_CONFIG_PATH:-config/base.yaml}
MAST3R_DEVICE=${MAST3R_DEVICE:-cuda:0}
MAST3R_IMAGE_SIZE=${MAST3R_IMAGE_SIZE:-512}
MAST3R_Q_CONF=${MAST3R_Q_CONF:-1.5}
MAST3R_MAX_MATCHES=${MAST3R_MAX_MATCHES:-600}

echo "  ROBOT_IDS: $ROBOT_IDS"
echo "  CLOUD_TOPIC_TEMPLATE: $CLOUD_TOPIC_TEMPLATE"
echo "  IMAGE_TOPIC_TEMPLATE: $IMAGE_TOPIC_TEMPLATE"
echo "  VERIFICATION_JOB_TOPIC: $VERIFICATION_JOB_TOPIC"
echo "  VERIFICATION_RESULT_TOPIC: $VERIFICATION_RESULT_TOPIC"
echo "  VERIFICATION_SUMMARY_TOPIC: $VERIFICATION_SUMMARY_TOPIC"
echo "  POSE_CONSTRAINT_TOPIC: $POSE_CONSTRAINT_TOPIC"
echo "  MIN_POINTS: $MIN_POINTS"
echo "  MAX_SUMMARY_POINTS: $MAX_SUMMARY_POINTS"
echo "  MAX_FEATURES: $MAX_FEATURES"
echo "  MAX_FEATURE_MATCHES: $MAX_FEATURE_MATCHES"
echo "  FEATURE_RATIO: $FEATURE_RATIO"
echo "  MAX_CORRESPONDENCE_PX: $MAX_CORRESPONDENCE_PX"
echo "  MIN_3D_CORRESPONDENCES: $MIN_3D_CORRESPONDENCES"
echo "  MIN_INLIERS: $MIN_INLIERS"
echo "  RANSAC_ITERATIONS: $RANSAC_ITERATIONS"
echo "  RANSAC_INLIER_THRESHOLD_M: $RANSAC_INLIER_THRESHOLD_M"
echo "  MAX_RMSE_M: $MAX_RMSE_M"
echo "  MIN_INLIER_RATIO: $MIN_INLIER_RATIO"
echo "  MIN_CLOUD_CONFIDENCE: $MIN_CLOUD_CONFIDENCE"
echo "  VERIFIER_BACKEND: $VERIFIER_BACKEND"
echo "  MAST3R_SLAM_ROOT: $MAST3R_SLAM_ROOT"
echo "  MAST3R_MODEL_PATH: $MAST3R_MODEL_PATH"
echo "  MAST3R_CONFIG_PATH: $MAST3R_CONFIG_PATH"
echo "  MAST3R_DEVICE: $MAST3R_DEVICE"
echo "  MAST3R_IMAGE_SIZE: $MAST3R_IMAGE_SIZE"
echo "  MAST3R_Q_CONF: $MAST3R_Q_CONF"
echo "  MAST3R_MAX_MATCHES: $MAST3R_MAX_MATCHES"

ROS_ARGS=(
    -p robot_ids:="$ROBOT_IDS"
    -p cloud_topic_template:="$CLOUD_TOPIC_TEMPLATE"
    -p image_topic_template:="$IMAGE_TOPIC_TEMPLATE"
    -p verification_job_topic:="$VERIFICATION_JOB_TOPIC"
    -p verification_result_topic:="$VERIFICATION_RESULT_TOPIC"
    -p verification_summary_topic:="$VERIFICATION_SUMMARY_TOPIC"
    -p pose_constraint_topic:="$POSE_CONSTRAINT_TOPIC"
    -p min_points:="$MIN_POINTS"
    -p max_summary_points:="$MAX_SUMMARY_POINTS"
    -p max_features:="$MAX_FEATURES"
    -p max_feature_matches:="$MAX_FEATURE_MATCHES"
    -p feature_ratio:="$FEATURE_RATIO"
    -p max_correspondence_px:="$MAX_CORRESPONDENCE_PX"
    -p min_3d_correspondences:="$MIN_3D_CORRESPONDENCES"
    -p min_inliers:="$MIN_INLIERS"
    -p ransac_iterations:="$RANSAC_ITERATIONS"
    -p ransac_inlier_threshold_m:="$RANSAC_INLIER_THRESHOLD_M"
    -p max_rmse_m:="$MAX_RMSE_M"
    -p min_inlier_ratio:="$MIN_INLIER_RATIO"
    -p min_cloud_confidence:="$MIN_CLOUD_CONFIDENCE"
    -p verifier_backend:="$VERIFIER_BACKEND"
    -p mast3r_slam_root:="$MAST3R_SLAM_ROOT"
    -p mast3r_config_path:="$MAST3R_CONFIG_PATH"
    -p mast3r_device:="$MAST3R_DEVICE"
    -p mast3r_image_size:="$MAST3R_IMAGE_SIZE"
    -p mast3r_q_conf:="$MAST3R_Q_CONF"
    -p mast3r_max_matches:="$MAST3R_MAX_MATCHES"
)

if [ -n "$MAST3R_MODEL_PATH" ]; then
    ROS_ARGS+=(-p mast3r_model_path:="$MAST3R_MODEL_PATH")
fi

ros2 run stretch3_ros_nodes multi_robot_geometric_verifier_node --ros-args "${ROS_ARGS[@]}"
