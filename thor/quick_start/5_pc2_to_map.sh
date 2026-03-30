#!/bin/bash
echo "Starting PC2 to Map Node..."
source /opt/ros/humble/setup.bash
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    source /workspace/thor/ros2_ws/install/setup.bash
fi
ros2 run stretch3_ros_nodes pc2_to_map_stretch3
