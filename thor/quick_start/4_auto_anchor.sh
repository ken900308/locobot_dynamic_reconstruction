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

echo "Starting Auto Anchor Node..."
source_ros_setup

ROBOT_ID=${ROBOT_ID:-robot1}
WORLD_FRAME=${WORLD_FRAME:-locobot/odom}
CAMERA_FRAME=${CAMERA_FRAME:-locobot/camera_color_optical_frame}
MAST3R_FRAME=${MAST3R_FRAME:-${ROBOT_ID}/mast3r_map}
POINTCLOUD_TOPIC=${POINTCLOUD_TOPIC:-/${ROBOT_ID}/mast3r/frame_pointcloud}
TF_TOPIC=${TF_TOPIC:-/${ROBOT_ID}/tf}
TF_STATIC_TOPIC=${TF_STATIC_TOPIC:-/${ROBOT_ID}/tf_static}

echo "  ROBOT_ID: $ROBOT_ID"
echo "  WORLD_FRAME: $WORLD_FRAME"
echo "  CAMERA_FRAME: $CAMERA_FRAME"
echo "  MAST3R_FRAME: $MAST3R_FRAME"
echo "  POINTCLOUD_TOPIC: $POINTCLOUD_TOPIC"
echo "  TF_TOPIC: $TF_TOPIC"
echo "  TF_STATIC_TOPIC: $TF_STATIC_TOPIC"

ros2 run stretch3_ros_nodes auto_anchor_from_pointcloud_stretch3 --ros-args \
    -p world_frame:="$WORLD_FRAME" \
    -p camera_frame:="$CAMERA_FRAME" \
    -p mast3r_frame:="$MAST3R_FRAME" \
    -p pointcloud_topic:="$POINTCLOUD_TOPIC" \
    -r /tf:="$TF_TOPIC" \
    -r /tf_static:="$TF_STATIC_TOPIC"
