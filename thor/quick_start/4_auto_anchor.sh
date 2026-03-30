#!/bin/bash
echo "Starting Auto Anchor Node..."
source /opt/ros/humble/setup.bash
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    source /workspace/thor/ros2_ws/install/setup.bash
fi
ros2 run stretch3_ros_nodes auto_anchor_from_pointcloud_stretch3 --ros-args -p world_frame:=locobot/odom
