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

echo "Starting ROSBridge Server..."
source_ros_setup
ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-9093}
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:="$ROSBRIDGE_PORT" max_message_size:=100000000
