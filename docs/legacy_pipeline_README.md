# 已封存：請勿作為現行操作文件

這份文件描述舊的 LoCoBot 2/4/5 pipeline，只供歷史追溯。
現行操作請回到上一層 README。

# MASt3R-SLAM x Locobot Jazzy

This repo runs the LoCoBot side of the MASt3R-SLAM reconstruction pipeline on the NVIDIA AGX Thor. The current pipeline is ROS-based and uses quick_start scripts `2`, `4`, and `5`.

The old IPC path is still present in the tree for reference, but it is not the active workflow.

## Current Pipeline

```text
LoCoBot robot ROS graph
  image / camera_info / tf relay
        |
        | rosbridge
        v
Thor MASt3R container
  quick_start/2_mast3r_slam.sh
        |
        +--> /robot1/mast3r/frame_pointcloud
        +--> /robot1/mast3r/pointcloud_in_map

  quick_start/4_auto_anchor.sh
        |
        +--> locobot/odom -> robot1/mast3r_map

  quick_start/5_pc2_to_map.sh
        |
        +--> /robot1/mast3r/pointcloud_in_map
```

## Robot Identity

LoCoBot is treated as `robot1`.

Default local Thor topics:

```text
/robot1/tf
/robot1/tf_static
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/pointcloud_in_map
```

Default TF frames:

```text
WORLD_FRAME=locobot/odom
CAMERA_FRAME=locobot/camera_color_optical_frame
MAST3R_FRAME=robot1/mast3r_map
```

The robot-side tf relay is expected to provide non-conflicting frame names before data reaches Thor.

## Rosbridge Inputs

`quick_start/2_mast3r_slam.sh` subscribes to the robot-side ROS graph through rosbridge.

Default robot-facing inputs:

```text
ROBOT_ROSBRIDGE_HOST=192.168.0.213
ROBOT_ROSBRIDGE_PORT=9090
ROBOT_IMAGE_TOPIC=/locobot/camera/camera/color/image_raw/compressed
ROBOT_CAMERA_INFO_TOPIC=/locobot/camera/camera/color/camera_info
ROBOT_TF_TOPIC=/tf
ROBOT_TF_STATIC_TOPIC=/locobot/tf_static_relay
```

`ROBOT_TF_STATIC_TOPIC` is the remote rosbridge source topic. The MASt3R node republishes that TF locally with ROS remapping:

```text
/tf        -> /robot1/tf
/tf_static -> /robot1/tf_static
```

This keeps robot1 TF isolated from robot2 when both pipelines run in the same ROS domain.

## Start Order

Run these inside the Thor container/workspace that has this repo mounted at `/workspace/thor`.

### Terminal 1: MASt3R-SLAM

```bash
cd /workspace/thor/quick_start
source 2_mast3r_slam.sh
```

Useful overrides:

```bash
source 2_mast3r_slam.sh --ip 192.168.0.213 --no-viz
source 2_mast3r_slam.sh --dds
source 2_mast3r_slam.sh --use-calib
```

### Terminal 2: Auto Anchor

```bash
cd /workspace/thor/quick_start
source 4_auto_anchor.sh
```

This waits for `/robot1/mast3r/frame_pointcloud`, looks up:

```text
locobot/odom -> locobot/camera_color_optical_frame
```

and publishes:

```text
locobot/odom -> robot1/mast3r_map
```

through `/robot1/tf_static`.

### Terminal 3: PointCloud To Map

```bash
cd /workspace/thor/quick_start
source 5_pc2_to_map.sh
```

This subscribes to:

```text
/robot1/mast3r/frame_pointcloud
/robot1/tf
/robot1/tf_static
```

and publishes:

```text
/robot1/mast3r/pointcloud_in_map
```

## Multi-Robot Notes

When LoCoBot and Stretch3 run in the same ROS domain:

```text
robot1 outputs:
  /robot1/mast3r/frame_pointcloud
  /robot1/mast3r/pointcloud_in_map
  /robot1/tf
  /robot1/tf_static

robot2 outputs:
  /robot2/mast3r/frame_pointcloud
  /robot2/mast3r/pointcloud_in_map
  /robot2/tf
  /robot2/tf_static
```

The local topic namespace separates transport. The TF frame prefixes from the robot-side relay separate frame names.
