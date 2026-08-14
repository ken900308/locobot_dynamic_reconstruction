#!/bin/bash
echo "Starting Milestone 1A Multi-Robot Fusion Node..."
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [ -f /workspace/thor/ros2_ws/install/setup.bash ]; then
    set +u
    source /workspace/thor/ros2_ws/install/setup.bash
    set -u
fi

GLOBAL_FRAME=${GLOBAL_FRAME:-global_map}
OUTPUT_TOPIC=${OUTPUT_TOPIC:-/multi_robot/global_pointcloud}
ROBOT_IDS=${ROBOT_IDS:-robot1,robot2}

ROBOT1_INPUT_TOPIC=${ROBOT1_INPUT_TOPIC:-/robot1/mast3r/frame_pointcloud}
ROBOT1_TRANSLATION=${ROBOT1_TRANSLATION:-0.0,-1.6,0.0}
ROBOT1_ROTATION_XYZW=${ROBOT1_ROTATION_XYZW:-0.0,0.0,0.7071068,0.7071068}

ROBOT2_INPUT_TOPIC=${ROBOT2_INPUT_TOPIC:-/robot2/mast3r/frame_pointcloud}
ROBOT2_TRANSLATION=${ROBOT2_TRANSLATION:-0.0,0.0,0.0}
ROBOT2_ROTATION_XYZW=${ROBOT2_ROTATION_XYZW:-0.0,0.0,0.0,1.0}

echo "  GLOBAL_FRAME: $GLOBAL_FRAME"
echo "  OUTPUT_TOPIC: $OUTPUT_TOPIC"
echo "  ROBOT_IDS: $ROBOT_IDS"
echo "  robot1 input: $ROBOT1_INPUT_TOPIC"
echo "  robot1 transform: t=[$ROBOT1_TRANSLATION], q=[$ROBOT1_ROTATION_XYZW]"
echo "  robot2 input: $ROBOT2_INPUT_TOPIC"
echo "  robot2 transform: t=[$ROBOT2_TRANSLATION], q=[$ROBOT2_ROTATION_XYZW]"

ros2 run stretch3_ros_nodes multi_robot_fusion_node --ros-args \
    -p global_frame:="$GLOBAL_FRAME" \
    -p output_topic:="$OUTPUT_TOPIC" \
    -p robot_ids:="$ROBOT_IDS" \
    -p robot1.input_topic:="$ROBOT1_INPUT_TOPIC" \
    -p robot1.translation:="$ROBOT1_TRANSLATION" \
    -p robot1.rotation_xyzw:="$ROBOT1_ROTATION_XYZW" \
    -p robot2.input_topic:="$ROBOT2_INPUT_TOPIC" \
    -p robot2.translation:="$ROBOT2_TRANSLATION" \
    -p robot2.rotation_xyzw:="$ROBOT2_ROTATION_XYZW"
