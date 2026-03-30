#!/bin/bash
# ROS2 rosbridge (multi-robot profile)
# - WebSocket port: 9091
# - Relaxed transport: max_message_size=100000000

source /opt/ros/humble/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

echo "Starting rosbridge_server on port 9091 (max_message_size=100000000)..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9091 \
  max_message_size:=100000000
