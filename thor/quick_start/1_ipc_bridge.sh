#!/bin/bash
echo "Starting IPC Bridge Node..."
source /opt/ros/humble/setup.bash
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    source /workspace/thor/ros2_ws/install/setup.bash
fi
ros2 run stretch3_ros_nodes ipc_bridge_node
