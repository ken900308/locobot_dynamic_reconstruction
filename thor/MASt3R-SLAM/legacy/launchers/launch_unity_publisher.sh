#!/bin/bash
"""
Unity Publisher Launcher
========================

Wrapper script to properly launch the Unity coordinate-corrected ROS2 publisher
"""

# Set up ROS2 environment
source /opt/ros/humble/setup.bash

# Set up Python path for MASt3R-SLAM
export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:${PYTHONPATH:-}"

# Change to correct directory
cd /workspace/MASt3R-SLAM

echo "🎮 Unity Point Cloud Publisher"
echo "==============================="
echo "Environment setup:"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  PYTHONPATH includes MASt3R-SLAM modules"
echo "  Unity coordinate conversion: ENABLED"
echo "  Performance optimizations: ENABLED"
echo ""

# Launch the Unity publisher with Unity coordinate fix
python3 simple_ply_ros2.py \
    --logs-dir /workspace/MASt3R-SLAM/logs \
    --pattern '*partial*.ply' \
    --unity-coords \
    --decimation 0.1 \
    --max-points 120000 \
    --poll-interval 0.5
