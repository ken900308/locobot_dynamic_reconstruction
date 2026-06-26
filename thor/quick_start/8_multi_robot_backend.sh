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

echo "Starting Multi-Robot Sim(3) Backend Node..."
source_ros_setup

ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}
METADATA_TOPIC_TEMPLATE=${METADATA_TOPIC_TEMPLATE:-}
if [ -z "$METADATA_TOPIC_TEMPLATE" ]; then
    METADATA_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_metadata'
fi
CLOUD_TOPIC_TEMPLATE=${CLOUD_TOPIC_TEMPLATE:-}
if [ -z "$CLOUD_TOPIC_TEMPLATE" ]; then
    CLOUD_TOPIC_TEMPLATE='/{robot_id}/mast3r/keyframe_cloud_local'
fi
CANDIDATE_TOPIC=${CANDIDATE_TOPIC:-/multi_robot/loop_candidates}
CANDIDATE_SUMMARY_TOPIC=${CANDIDATE_SUMMARY_TOPIC:-/multi_robot/loop_candidate_summaries}
VERIFICATION_JOB_TOPIC=${VERIFICATION_JOB_TOPIC:-/multi_robot/geometric_verification_jobs}
MIN_SIMILARITY=${MIN_SIMILARITY:-0.82}
TOP_K=${TOP_K:-3}

echo "  ROBOT_IDS: $ROBOT_IDS"
echo "  METADATA_TOPIC_TEMPLATE: $METADATA_TOPIC_TEMPLATE"
echo "  CLOUD_TOPIC_TEMPLATE: $CLOUD_TOPIC_TEMPLATE"
echo "  CANDIDATE_TOPIC: $CANDIDATE_TOPIC"
echo "  CANDIDATE_SUMMARY_TOPIC: $CANDIDATE_SUMMARY_TOPIC"
echo "  VERIFICATION_JOB_TOPIC: $VERIFICATION_JOB_TOPIC"
echo "  MIN_SIMILARITY: $MIN_SIMILARITY"
echo "  TOP_K: $TOP_K"

ros2 run stretch3_ros_nodes multi_robot_backend_node --ros-args \
    -p robot_ids:="$ROBOT_IDS" \
    -p metadata_topic_template:="$METADATA_TOPIC_TEMPLATE" \
    -p cloud_topic_template:="$CLOUD_TOPIC_TEMPLATE" \
    -p candidate_topic:="$CANDIDATE_TOPIC" \
    -p candidate_summary_topic:="$CANDIDATE_SUMMARY_TOPIC" \
    -p verification_job_topic:="$VERIFICATION_JOB_TOPIC" \
    -p min_similarity:="$MIN_SIMILARITY" \
    -p top_k:="$TOP_K"
