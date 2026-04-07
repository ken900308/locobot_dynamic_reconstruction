#!/bin/bash
echo "Starting MASt3R-SLAM with Visualization..."
# Usage: source 2_mast3r_slam.sh [--viz | --no-viz] [--use-calib | --no-calib]
# --use-calib : enable camera intrinsics from config/intrinsics.yaml (locobot D435)
# --no-calib  : let MASt3R self-calibrate (default)
/workspace/thor/MASt3R-SLAM/launch_mast3r_visual_ros2_igbr.sh --viz --use-calib
# /workspace/thor/MASt3R-SLAM/launch_mast3r_visual_ros2_igbr.sh --viz --no-calib

