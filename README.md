# MASt3R-SLAM × Locobot — 即時場景重建系統

本專案在 NVIDIA AGX Thor 上運行 MASt3R-SLAM，透過 IPC 橋接將點雲即時推送到 Unity 進行可視化，採用 **IBGR（Incremental Build, Global Redraw）** 雙緩衝架構。

系統由兩個 Docker 容器組成：

| 容器名稱                                    | 功能                                                |
| ------------------------------------------- | --------------------------------------------------- |
| `mast3r_locobot_algorithm` (GPU Container)  | MASt3R-SLAM 演算法核心，負責影像推理與 PGO          |
| `mast3r_locobot_ros_humble` (ROS Container) | IPC 橋接、TF 建立、點雲轉換、rosbridge 廣播至 Unity |

---

## 🚀 啟動流程

### Step 1：啟動 Docker 容器

在 host machine 上，開兩個terminal，分別`cd`到專案根目錄後執行：

```bash
cd /path_to_where_you_clone_this_repo/locobot/docker

# 啟動 ROS2 Humble 容器
./run_mast3r_arm_humble_locobot.sh
```

```bash
cd /path_to_where_you_clone_this_repo/locobot/docker

# 啟動 GPU (Algorithm) 容器
./run_mast3r_arm_algorithm_locobot.sh
```

---

### Step 2：設定 ROS Domain ID（每個容器進入後先做）

進入任何一個容器後，**都必須先 source environment**：

```bash
source /workspace/thor/environment.sh 150
```

> `150` 為 ROS_DOMAIN_ID，確保與 locobot 機器人在同一個 ROS2 網路分區。

---

### Step 3：依序開啟 6 個 Terminal

所有腳本均位於 `/workspace/thor/quick_start/`。

---

#### Terminal 1 — IPC Bridge（ROS 容器）

接收 locobot 相機的影像並轉發給 GPU 容器。

```bash
# 進入 ROS 容器
docker exec -it mast3r_locobot_ros_humble bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 1_ipc_bridge.sh
```

---

#### Terminal 2 — MASt3R-SLAM（GPU 容器 ⚠️）

> **注意：這個腳本必須在 GPU 容器裡執行！**

```bash
# 進入 GPU 容器
docker exec -it mast3r_locobot_algorithm bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 2_mast3r_slam.sh
```

---

#### Terminal 3 — IPC Pointcloud Receiver（ROS 容器）

從 GPU 容器的 Unix Socket 接收點雲，發布到 ROS topic `/mast3r/frame_pointcloud`。

```bash
docker exec -it mast3r_locobot_ros_humble bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 3_ipc_receiver.sh
```

---

#### Terminal 4 — Auto Anchor（ROS 容器）

接收第一筆點雲後，建立 `locobot/odom → mast3r_map` 的靜態 TF，作為整個地圖的坐標系錨點。

```bash
docker exec -it mast3r_locobot_ros_humble bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 4_auto_anchor.sh
```

---

#### Terminal 5 — PC2 to Map（ROS 容器）

將點雲從 `mast3r_map` 座標轉換到 `locobot/odom`，附加位姿資訊後發布至 `/mast3r/pointcloud_in_map`。

```bash
docker exec -it mast3r_locobot_ros_humble bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 5_pc2_to_map.sh
```

---

#### Terminal 6 — Rosbridge（ROS 容器）

啟動 WebSocket server，Unity 透過此橋接訂閱點雲 topic。

```bash
docker exec -it mast3r_locobot_ros_humble bash
source /workspace/thor/environment.sh 150
cd /workspace/thor/quick_start
source 6_rosbridge.sh
```

---

## 📡 資料流

```text
Locobot 相機
    │ ROS2 image topic
    ▼
[Terminal 1] ipc_bridge_node (ROS 容器)
    │ Unix Domain Socket (IPC)
    ▼
[Terminal 2] MASt3R-SLAM (GPU 容器)  ← 演算法在這裡跑
    │ Unix Domain Socket (IPC)
    ▼
[Terminal 3] ipc_pointcloud_receiver → /mast3r/frame_pointcloud
    │
[Terminal 4] auto_anchor  (建立 locobot/odom → mast3r_map TF)
    │
[Terminal 5] pc2_to_map  → /mast3r/pointcloud_in_map
    │ WebSocket (rosbridge)
    ▼
Unity (PointCloudAccumulatorGPU_Anchored_IBGR.cs)
```

---

## ⚙️ Unity 設定

- Unity 訂閱 topic：`/mast3r/pointcloud_in_map`
- Rosbridge WebSocket 位址：`ws://<Thor IP>:9091`
- 使用腳本：`PointCloudAccumulatorGPU_Anchored_IBGR.cs`
