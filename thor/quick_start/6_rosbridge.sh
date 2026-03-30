#!/bin/bash
echo "Starting ROSBridge Server..."
source /opt/ros/humble/setup.bash
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    source /workspace/thor/ros2_ws/install/setup.bash
fi
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091 max_message_size:=100000000
