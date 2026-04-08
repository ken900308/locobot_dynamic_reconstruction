#!/bin/bash
echo "Starting ROS 2 bag recording for topics from ipc_bridge"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
BAG_NAME="locobot_camera_$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${SCRIPT_DIR}/${BAG_NAME}"

echo "Saving bag to: ${BAG_PATH}"
ros2 bag record -o "${BAG_PATH}" /locobot/camera/camera/color/image_raw/compressed /locobot/camera/camera/color/camera_info /tf /tf_static
