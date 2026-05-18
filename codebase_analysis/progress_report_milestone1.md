# Milestone 1A 進度報告

## 範圍

Milestone 1A 目前實作為 **基於手動 / static transform 的多機器人點雲融合**。

這個 milestone 尚未實作：

- AprilTag encounter transform estimation
- visual keyframe matching
- shared retrieval database
- cross-robot PGO
- Sim3 scale refinement

本階段目標是先驗證第一層整合流程：

```text
robot1 MASt3R-SLAM pointclouds
robot2 MASt3R-SLAM pointclouds
manual per-robot transform
fused global pointcloud topic
Unity/global visualization
```

也就是先確認兩台 robot 的 reconstruction output 可以被分開接收、套用已知 transform，並發布成同一個 global fused pointcloud。

---

## 已完成更動

### 1. locobot quick_start 已加入 robot1 預設設定

修改檔案：

- `/home/hrc/Desktop/projects/locobot/thor/quick_start/1_ipc_bridge.sh`
- `/home/hrc/Desktop/projects/locobot/thor/quick_start/2_mast3r_slam.sh`
- `/home/hrc/Desktop/projects/locobot/thor/quick_start/3_ipc_receiver.sh`
- `/home/hrc/Desktop/projects/locobot/thor/quick_start/4_auto_anchor.sh`
- `/home/hrc/Desktop/projects/locobot/thor/quick_start/5_pc2_to_map.sh`

預設 robot identity：

```text
ROBOT_ID=robot1
```

預設 socket paths：

```text
/tmp/ipc_socket/robot1/mast3r_image.sock
/tmp/ipc_socket/robot1/mast3r_pointcloud.sock
```

預設 pointcloud topics：

```text
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/keyframe_pose_updates
/robot1/mast3r/pointcloud_in_map
```

以上重要參數都可以在執行 quick_start script 前用環境變數覆蓋。

---

### 2. Stretch3 quick_start 已加入 robot2 預設設定

修改檔案：

- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start/1_ipc_bridge.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start/2_mast3r_slam.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start/3_ipc_receiver.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start/4_auto_anchor.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start/5_pc2_to_map.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/MASt3R-SLAM/launch_mast3r_visual_ros2_igbr.sh`
- `/home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/MASt3R-SLAM/mast3r_slam_visual_IGBR.py`

預設 robot identity：

```text
ROBOT_ID=robot2
```

預設 socket paths：

```text
/tmp/ipc_socket/robot2/mast3r_image.sock
/tmp/ipc_socket/robot2/mast3r_pointcloud.sock
```

Stretch3 的 quick_start、MASt3R launch script、以及 Python IPC fallback 現在都使用同一個 `ROBOT_ID=robot2` 規則產生 socket path，避免 quick_start 與直接啟動 MASt3R 時出現不同 IPC namespace。

預設 pointcloud topics：

```text
/robot2/mast3r/frame_pointcloud
/robot2/mast3r/keyframe_pose_updates
/robot2/mast3r/pointcloud_in_map
```

Stretch3 的 TF world frame 預設為 `odom`。實機 ROS graph 中沒有 `stretch3/odom` 這個 TF frame，因此 `4_auto_anchor.sh` 與 `5_pc2_to_map.sh` 都使用 `WORLD_FRAME=odom`。

---

### 3. 新增 Milestone 1A fusion node

新增檔案：

- `/home/hrc/Desktop/projects/locobot/thor/ros2_ws/src/stretch3_ros_nodes/stretch3_ros_nodes/multi_robot_fusion_node.py`
- `/home/hrc/Desktop/projects/locobot/thor/quick_start/7_multi_robot_fusion.sh`

修改 ROS package entry point：

- `/home/hrc/Desktop/projects/locobot/thor/ros2_ws/src/stretch3_ros_nodes/setup.py`

fusion node 目前設計為透過以下方式啟動：

```bash
ros2 run stretch3_ros_nodes multi_robot_fusion_node
```

fusion node 功能：

1. 訂閱 namespaced MASt3R keyframe pointcloud topics。
2. 對每台 robot 套用手動 / static transform。
3. 將轉換後的 pointcloud 發布到共用 global topic。

預設輸入：

```text
/robot1/mast3r/frame_pointcloud
/robot2/mast3r/frame_pointcloud
```

預設輸出：

```text
/multi_robot/global_pointcloud
```

預設 manual transforms：

```text
robot1:
  translation = 0.0,0.0,0.0
  rotation_xyzw = 0.0,0.0,0.0,1.0

robot2:
  translation = 2.0,0.0,0.0
  rotation_xyzw = 0.0,0.0,0.0,1.0
```

也就是目前預設把 robot1 視為 global origin，robot2 放在 +X 方向 2 公尺的位置。

---

## 新增需要啟動的 program

Milestone 1A 新增一個 quick_start program：

```text
/home/hrc/Desktop/projects/locobot/thor/quick_start/7_multi_robot_fusion.sh
```

建議在兩台 robot 的 pointcloud receivers 都開始發布後，再從 locobot 端啟動。

預設啟動方式：

```bash
cd /home/hrc/Desktop/projects/locobot/thor/quick_start
./7_multi_robot_fusion.sh
```

這個 script 內部會呼叫：

```bash
ros2 run stretch3_ros_nodes multi_robot_fusion_node
```

因為 fusion node 已經註冊成 ROS package console script，所以更新後需要重新 build 並 source locobot 的 ROS workspace：

```bash
cd /home/hrc/Desktop/projects/locobot/thor/ros2_ws
colcon build --packages-select stretch3_ros_nodes
source install/setup.bash
```

如果要修改 robot2 的手動 transform，可以這樣啟動：

```bash
ROBOT2_TRANSLATION=1.5,0.0,0.0 \
ROBOT2_ROTATION_XYZW=0.0,0.0,0.0,1.0 \
./7_multi_robot_fusion.sh
```

---

## 建議 Milestone 1A 啟動順序

和原本 quick_start 使用方式一樣，建議每個 program 開在不同 terminal。

### Robot1：locobot

```bash
cd /home/hrc/Desktop/projects/locobot/thor/quick_start
./1_ipc_bridge.sh
./2_mast3r_slam.sh
./3_ipc_receiver.sh
```

可選的 robot1 local map utilities：

```bash
./4_auto_anchor.sh
./5_pc2_to_map.sh
```

### Robot2：Stretch3

```bash
cd /home/hrc/Desktop/projects/Strethc3_Dynamic_Reconstruction/thor/quick_start
./1_ipc_bridge.sh
./2_mast3r_slam.sh
./3_ipc_receiver.sh
```

可選的 robot2 local map utilities：

```bash
./4_auto_anchor.sh
./5_pc2_to_map.sh
```

### Global fusion

```bash
cd /home/hrc/Desktop/projects/locobot/thor/quick_start
./7_multi_robot_fusion.sh
```

### ROSBridge

如果 Unity / global visualization 不需要分別連兩個 rosbridge，Milestone 1A 建議先只開一個 rosbridge。

從 locobot 端啟動：

```bash
cd /home/hrc/Desktop/projects/locobot/thor/quick_start
./6_rosbridge.sh
```

Unity / visualization 端要看的 fused pointcloud topic 是：

```text
/multi_robot/global_pointcloud
```

---

## 重要注意事項

### 目前 fusion output convention

fusion node 會把 XYZ points 轉換到共用 global coordinate convention，但目前仍把 robot / keyframe identity 保留在 `PointCloud2.header.frame_id`：

```text
robot1_kf_42
robot2_kf_17
```

這是為了相容目前 Unity pipeline，因為 Unity 端已經會用 `frame_id` 判斷 keyframe identity。

長期來說，這些 metadata 應該改成獨立 structured metadata topic，而不是塞在 `frame_id` 裡。

### 目前只支援 manual transform

Milestone 1A 假設 inter-map transform 已知，並由使用者手動給定。

目前尚未估計：

```text
T_robot1_map_robot2_map
```

AprilTag-based transform estimation 應該放在 Milestone 1B。

### 目前沒有 PGO

目前實作只做 pointcloud transform 和 republish。

此階段沒有修改 MASt3R-SLAM 的 `SharedKeyframes`、`FactorGraph` 或 backend PGO。

---

## 已完成驗證

已對更新後的 shell scripts 做語法檢查：

```text
bash -n locobot quick_start scripts
bash -n Stretch3 quick_start scripts
```

已對 fusion node 做 Python compile check：

```text
stretch3_ros_nodes/multi_robot_fusion_node.py
```

以上語法檢查皆通過。
