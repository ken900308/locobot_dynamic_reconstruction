#!/bin/bash
set -euo pipefail

source_ros_setup() {
    set +u
    source /opt/ros/humble/setup.bash
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
MIN_CLOUD_CONFIDENCE=${MIN_CLOUD_CONFIDENCE:-0.0}

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
echo "  MIN_CLOUD_CONFIDENCE: $MIN_CLOUD_CONFIDENCE"

ros2 run stretch3_ros_nodes multi_robot_geometric_verifier_node --ros-args \
    -p robot_ids:="$ROBOT_IDS" \
    -p cloud_topic_template:="$CLOUD_TOPIC_TEMPLATE" \
    -p image_topic_template:="$IMAGE_TOPIC_TEMPLATE" \
    -p verification_job_topic:="$VERIFICATION_JOB_TOPIC" \
    -p verification_result_topic:="$VERIFICATION_RESULT_TOPIC" \
    -p verification_summary_topic:="$VERIFICATION_SUMMARY_TOPIC" \
    -p pose_constraint_topic:="$POSE_CONSTRAINT_TOPIC" \
    -p min_points:="$MIN_POINTS" \
    -p max_summary_points:="$MAX_SUMMARY_POINTS" \
    -p max_features:="$MAX_FEATURES" \
    -p max_feature_matches:="$MAX_FEATURE_MATCHES" \
    -p feature_ratio:="$FEATURE_RATIO" \
    -p max_correspondence_px:="$MAX_CORRESPONDENCE_PX" \
    -p min_3d_correspondences:="$MIN_3D_CORRESPONDENCES" \
    -p min_inliers:="$MIN_INLIERS" \
    -p ransac_iterations:="$RANSAC_ITERATIONS" \
    -p ransac_inlier_threshold_m:="$RANSAC_INLIER_THRESHOLD_M" \
    -p max_rmse_m:="$MAX_RMSE_M" \
    -p min_cloud_confidence:="$MIN_CLOUD_CONFIDENCE"
