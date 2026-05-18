# dev_goal.md

# 多機器人環境重建開發目標

## 1. 最終目標

本專案的最終目標是建立一套 **多機器人協同環境重建系統**。系統中會有至少兩台機器人，例如 `robot1` 與 `robot2`。兩台機器人會分別在不同區域中執行 MASt3R-SLAM，並各自重建所在場景。

在初始階段，兩台機器人彼此不知道對方在真實空間中的位置。因此，兩台機器人重建出的環境會各自存在於自己的 local map frame 中。當兩台機器人相遇，或觀測到足夠的共同場景資訊後，系統會估計兩張 local map 之間的相對座標轉換，並將兩台機器人各自重建出的 local environment 拼接成一張 unified global map。

相遇後的拼接區域不應只是單純把兩份點雲疊在一起，而是需要建立跨機器人的 keyframe constraint，並在拼接處執行 pose graph optimization，使兩張 local reconstruction 能夠在共同區域附近達到一致的幾何對齊。

簡化後的概念如下：

```text
相遇前：

robot1 local map:
  r1_kf_1 -> r1_kf_2 -> r1_kf_3 -> ...

robot2 local map:
  r2_kf_a -> r2_kf_b -> r2_kf_c -> ...

此時 robot1 map 與 robot2 map 之間沒有已知 transform。
```

```text
相遇後：

估計：
  T_global_robot1_map
  T_global_robot2_map

或等價地估計：
  T_robot1_map_robot2_map

接著融合：
  robot1 local map + transformed robot2 local map -> unified global map

最後：
  在相遇拼接處執行 PGO，並更新 global reconstruction。
```

---

## 2. 核心系統概念

本系統應採用 **encounter-triggered map fusion** 的設計概念，也就是「相遇觸發式地圖融合」。

整體流程可以分成四個階段：

### 2.1 各自進行 local reconstruction

每台機器人各自獨立執行 MASt3R-SLAM，並建立自己的 local map。

```text
robot1:
  image stream -> MASt3R-SLAM -> robot1 local keyframes / pointmaps / poses

robot2:
  image stream -> MASt3R-SLAM -> robot2 local keyframes / pointmaps / poses
```

在這個階段中，兩張 local map 尚未被連接，因為兩個 local map frame 之間的 transform 仍然未知。

### 2.2 相遇前的資料共享

雖然兩台機器人在一開始不知道彼此的空間關係，但它們仍然可以透過 ROS 2 topic 或 IPC / network bridge 交換資料。

可能共享的資料包含：

- keyframe images
- keyframe IDs
- 每個 keyframe 在該機器人 local map 中的 pose
- keyframe pointclouds / pointmaps
- visual descriptors 或 retrieval features
- robot TF information
- encounter detection information，例如 AprilTag detection 或 visual overlap candidates

不過，在兩張 map 之間的相對 transform 被估計出來之前，收到的資料只能先被 buffer 或建立索引，不應該直接插入 Unity 的 global map 中。

### 2.3 相遇偵測與座標轉換

當兩台機器人相遇時，系統需要估計兩張 local map 之間的空間關係。

相遇可以透過不同方法被偵測：

#### 方法 A：AprilTag-based encounter

如果其中一台機器人觀測到另一台機器人身上的 AprilTag，系統可以估計兩台機器人 base frame 之間的 relative pose。

範例 transform chain：

```text
robot1_map -> robot1_base
robot1_base -> robot1_camera
robot1_camera -> tag_on_robot2
robot2_base -> tag_on_robot2
robot2_map -> robot2_base
```

根據這條 transform chain，可以推導出：

```text
T_robot1_map_robot2_map
```

一個常用形式為：

```text
T_robot1_map_robot2_map
= T_robot1_map_robot1_base
  * T_robot1_base_robot2_base
  * inverse(T_robot2_map_robot2_base)
```

這個 transform 會成為兩張 local map 之間的初始座標轉換。

#### 方法 B：Visual keyframe matching

如果兩台機器人沒有直接看到彼此，但它們曾經觀測到同一個場景，系統可以跨機器人搜尋視覺上相似的 keyframes。

例如：

```text
robot1_kf_5 <-> robot2_kf_c
```

接著可以使用 MASt3R-style pointmap matching 或 feature matching 來估計兩個 keyframes 之間的相對 transform。

#### 方法 C：混合式方法

實務上第一版可以使用 AprilTag-based encounter detection 來提高穩定性，後續再加入 visual keyframe retrieval 作為 refinement 或研究貢獻。

### 2.4 Map Fusion 與 PGO

當兩張 local map 之間的 transform 被估計出來後，robot2 在相遇前與相遇後產生的 keyframes 都可以被轉換到 global map frame。

重要觀念：

系統不應該只處理相遇之後產生的新 keyframes，也必須處理 robot2 在相遇之前已經建立好的 keyframes。

因此，每台機器人都應該維護一個 keyframe buffer：

```text
robot2_keyframe_buffer = [r2_kf_a, r2_kf_b, r2_kf_c, ...]
```

當 encounter transform 可用時：

```text
for each keyframe in robot2_keyframe_buffer:
    transform keyframe pointmap into global frame
    publish / insert into unified map
```

完成初始融合後，相遇區域應該被加入為 pose graph 中的一條 cross-robot constraint。

範例 graph structure：

```text
robot1 internal edges:
  r1_kf_1 -> r1_kf_2 -> r1_kf_3 -> r1_kf_4 -> r1_kf_5

robot2 internal edges:
  r2_kf_a -> r2_kf_b -> r2_kf_c -> r2_kf_d

cross-robot encounter edge:
  r1_kf_5 <-> r2_kf_d
```

後端應該使用 intra-robot edges 與 cross-robot encounter edge 共同執行 PGO。

---

## 3. 主要設計問題一：各自開 process 還是跑在同一個 process？

一個關鍵架構選擇是：每台機器人是否應該各自執行自己的 MASt3R-SLAM process，或者所有機器人的 image streams 都送進同一個 centralized reconstruction process。

---

### 3.1 選項 A：每台機器人各自執行自己的 MASt3R-SLAM process

在這個設計中，每台機器人獨立執行一個 MASt3R-SLAM instance。

```text
robot1 image stream -> robot1 MASt3R-SLAM process -> robot1 local map
robot2 image stream -> robot2 MASt3R-SLAM process -> robot2 local map
```

另外設計一個獨立的 fusion node，負責收集兩台機器人的輸出，並處理 encounter detection、coordinate transfer、keyframe buffering、map fusion 與 global PGO。

```text
robot1 MASt3R-SLAM output ┐
                          ├── multi_robot_fusion_node -> unified global map
robot2 MASt3R-SLAM output ┘
```

#### 優點

- 模組化程度高，較容易 debug。
- 即使其中一台機器人斷線，另一台仍可繼續重建。
- 更符合真實多機器人部署情境。
- 未來比較容易擴展到更多台機器人。
- 每台機器人可以使用自己的 IPC socket、camera topic 與 local TF tree。

#### 缺點

- 必須額外設計 data sharing 機制。
- 需要嚴格管理 namespace，例如 `/robot1/...` 與 `/robot2/...`。
- 需要 map fusion layer 管理不同 local maps 之間的 transform。
- cross-robot PGO 會更複雜，因為每台機器人都有自己的 local pose graph。
- 如果每台機器人的 monocular reconstruction scale 不一致，單純 SE(3) transform 可能不足，可能需要 Sim(3) transform。

#### 需要共享的資料

每個 MASt3R-SLAM process 應該輸出足夠的資訊給 fusion node：

```text
/robot1/mast3r/keyframes
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/keyframe_poses
/robot1/mast3r/descriptors

/robot2/mast3r/keyframes
/robot2/mast3r/frame_pointcloud
/robot2/mast3r/keyframe_poses
/robot2/mast3r/descriptors
```

fusion node 訂閱兩台機器人的輸出，維護 keyframe buffer，並發布 unified map：

```text
/multi_robot/global_pointcloud
/multi_robot/global_keyframe_graph
/multi_robot/global_pose_updates
```

這會是比較推薦的第一版架構，因為它保留了機器人的獨立性，同時允許後續進行 map fusion。

---

### 3.2 選項 B：兩台機器人跑在同一個 shared MASt3R-SLAM process 中

在這個設計中，兩台機器人的影像都送進同一個 process。

```text
robot1 image stream ┐
                    ├── shared MASt3R-SLAM process -> global map
robot2 image stream ┘
```

#### 優點

- 只需要維護一個 global keyframe database。
- PGO 從一開始就可以在同一個 backend graph 中處理。
- 不需要事後合併兩個獨立 pose graphs。
- 因為所有 keyframes 都在同一個 process 中，cross-robot keyframe selection 可能較容易實作。

#### 缺點

- 較難部署到真實多機器人系統。
- centralized process 會成為系統瓶頸。
- 如果網路中斷，兩台機器人的重建都可能被影響。
- 需要處理多個 image streams 的同步問題與 robot identity 管理。
- 可能需要深度修改 MASt3R-SLAM，讓它支援 multi-agent input streams。
- 不太符合 distributed robotic system 的設計精神。

這種設計從 global optimization 的角度很乾淨，但作為第一版實作難度較高。

---

### 3.3 目前建議

第一版建議採用：

```text
每台機器人各自執行 MASt3R-SLAM process
+ 共用一個 multi_robot_fusion_node
```

這樣可以形成清楚的開發路徑：

1. 保留每台機器人的獨立 reconstruction pipeline。
2. 加入 robot 之間的 ROS 2 topic sharing。
3. encounter 前先 buffer remote keyframes。
4. encounter 後估計 inter-map transform。
5. 將 remote keyframes 轉換到 global frame。
6. 發布 unified map 給 Unity。
7. 在 encounter edge 附近加入 PGO。

---

## 4. 主要設計問題二：獨立 process 之間如何共享資料？

如果每台機器人各自執行自己的 process，那麼資料共享必須被明確設計。

可能方式如下：

### 4.1 ROS 2 topic sharing

每台機器人在自己的 namespace 底下發布 MASt3R-SLAM output。

例如：

```text
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/keyframe_pose
/robot1/mast3r/image

/robot2/mast3r/frame_pointcloud
/robot2/mast3r/keyframe_pose
/robot2/mast3r/image
```

fusion node 訂閱兩個 namespace 底下的資料。

這個方式簡單，而且與目前 ROS 2 系統相容。

### 4.2 每台機器人使用獨立 IPC socket

每台機器人應該使用不同的 IPC socket path，避免資料互相污染。

例如：

```text
/tmp/ipc_socket/robot1/mast3r_image.sock
/tmp/ipc_socket/robot1/mast3r_pointcloud.sock

/tmp/ipc_socket/robot2/mast3r_image.sock
/tmp/ipc_socket/robot2/mast3r_pointcloud.sock
```

這樣可以避免 robot1 的 MASt3R process 誤吃到 robot2 的 image stream。

### 4.3 ROS + IPC 混合架構

一個實用的設計是：

```text
Camera image -> ROS 2 topic -> IPC bridge -> MASt3R-SLAM process
MASt3R output -> IPC receiver -> ROS 2 topic -> fusion node
```

每台機器人都有自己的 socket namespace 與 ROS namespace。

這與目前系統架構接近，應該會比較容易擴展。

---

## 5. Unity / Digital Twin 考量

重建結果會在 Unity 中呈現為 digital twin。這會帶來一個重要挑戰：

系統必須同時支援：

1. 每台機器人獨立重建時的即時視覺化。
2. 機器人相遇後的 coordinate transfer 與 map fusion update。

在 encounter 發生前，Unity 不能假設 robot1 map 與 robot2 map 已經在同一個 global frame 中。

因此，Unity visualization 應該依照 map state 設計。

---

### 5.1 相遇前：分離的 local digital twins

在 encounter transform 被估計出來之前，Unity 可以用兩種方式顯示兩台機器人的重建結果。

#### 選項 A：只顯示目前 active robot 的 local map

這是最簡單的模式。

```text
Unity displays robot1 local map
或
Unity displays robot2 local map
```

使用者可以切換不同機器人的視角，但兩張地圖尚未被空間融合。

#### 選項 B：同時顯示兩張 local maps，但放在暫時的 debug 空間

Unity 可以將兩張 local maps 顯示在不同位置，並用人工 offset 將它們分開。

例如：

```text
robot1 local map 顯示在 Unity origin
robot2 local map 顯示在旁邊 10 meters 的位置
```

這種方式適合 debug，但不應該被視為真實的 global reconstruction。

---

### 5.2 相遇時：coordinate transfer

當 encounter transform 被估計出來後，系統必須執行一次 coordinate transfer。

系統應該計算：

```text
T_global_robot1_map
T_global_robot2_map
```

或者選擇 robot1 map 作為 global frame：

```text
T_global_robot1_map = Identity
T_global_robot2_map = T_robot1_map_robot2_map
```

接著，robot2 的所有 keyframes 與 pointclouds 就可以被轉換到 robot1/global frame 中。

這個步驟如果直接在 Unity 畫面上改動，可能會造成地圖突然跳動。因此 Unity 最好接收後端產生的 corrected full map update，而不是在畫面上手動移動個別點。

---

### 5.3 相遇後：統一的 global digital twin

coordinate transfer 完成後，Unity 應該切換到 unified global map representation。

推薦視覺化策略：

```text
相遇前：
  分開渲染 local maps，或只渲染 active robot map

相遇後：
  接收 fused global map
  切換到 unified global map
  後續新 keyframes 直接 append 到 global frame
```

既有的 IBGR-style 設計很適合這個情境：

```text
Incremental Build:
  持續 append 新的 keyframe pointclouds，維持流暢視覺化

Global Redraw:
  當 fusion 或 PGO 更新地圖時，將完整修正後的 map 分 chunk 傳送
  Unity 在 back buffer 中接收 chunks
  當所有 chunks 到齊後，再 swap 到 corrected global map
```

這樣可以維持 digital twin 的連續性，因為 Unity 在後端準備 corrected fused map 時，仍然可以繼續顯示舊地圖。

---

## 6. 重要技術議題

### 6.1 Frame 管理

每台機器人都應該有清楚分離的 frame names。

例如：

```text
global_map

robot1/map
robot1/base_link
robot1/camera_link
robot1/camera_color_optical_frame
robot1/mast3r_map

robot2/map
robot2/base_link
robot2/camera_link
robot2/camera_color_optical_frame
robot2/mast3r_map
```

相遇前：

```text
global_map -> robot1/mast3r_map may exist
robot2/mast3r_map is not connected to global_map
```

相遇後：

```text
global_map -> robot2/mast3r_map is published
```

### 6.2 Keyframe identity

融合後不應該把 robot2 的 keyframes 重新命名成 robot1 的 keyframes。

應使用 namespaced IDs：

```text
robot1_kf_0001
robot1_kf_0002
robot2_kf_0001
robot2_kf_0002
```

fused graph 應該保留 robot identity。

### 6.3 SE(3) vs Sim(3)

如果兩台機器人的 maps 都有一致的 metric scale，那麼 SE(3) 可能足夠：

```text
rotation + translation
```

然而，由於 MASt3R-SLAM 是基於 monocular reconstruction 的系統，不同 local maps 可能存在 scale inconsistency。在這種情況下，inter-map transform 應該使用 Sim(3)：

```text
scale + rotation + translation
```

第一版可以先使用 robot pose / AprilTag 得到的 SE(3) 作為 initial alignment。後續再考慮加入 Sim(3) refinement。

### 6.4 Encounter area 的 PGO

融合不應該只停在 coordinate transfer。

相遇時應該建立一條 cross-robot edge：

```text
robot1_kf_i <-> robot2_kf_j
```

接著 PGO 應該針對 encounter area 附近的 local graph 進行優化，或最終擴展成 full global graph optimization。

第一版可以只在局部 window 中執行 PGO：

```text
robot1 recent keyframes + robot2 recent keyframes + encounter edge
```

後續版本再擴展到整張 graph。

### 6.5 Encounter 前的 buffered keyframes

相遇前，來自遠端機器人的 keyframes 應該先被儲存，但不要直接融合。

相遇後，所有 buffered remote keyframes 都應該被轉換並插入 global map。

這是必要的，因為 robot2 在遇到 robot1 之前，可能已經重建了一段有價值的環境。

### 6.6 Digital twin continuity

Unity digital twin 應避免長時間空白畫面或破壞性的突然更新。

推薦做法：

- backend fusion 執行時，Unity 繼續渲染目前地圖
- corrected global map 以 chunked full-map update 形式送到 Unity
- Unity 使用 double buffering
- 所有 chunks 到齊後再切換到 corrected map

---

## 7. 建議第一階段里程碑

第一階段不應該一開始就嘗試完整 multi-robot PGO。

第一階段應先證明兩台獨立重建的 local maps 可以在 encounter 後融合。

### Milestone 1：不含完整 PGO 的 encounter-based map fusion

目標：

```text
robot1 與 robot2 各自獨立重建 local maps。
當 encounter transform 可用後，將 robot2 buffered keyframes 轉換到 robot1/global frame，並在 Unity 中顯示為一張地圖。
```

需要元件：

1. 兩套獨立 MASt3R-SLAM pipelines。
2. robot1 與 robot2 的 namespaced ROS topics。
3. 每台機器人的 keyframe pointcloud buffer。
4. encounter transform estimation。
5. 將 remote keyframes 轉換到 global frame。
6. 發布 fused pointcloud 給 Unity。
7. Unity 顯示 fused map。

PGO 可以在這個 milestone 成功後再加入。

---

## 8. 建議第二階段里程碑

### Milestone 2：Encounter edge 與 local PGO

目標：

```text
encounter 發生後，建立 cross-robot constraint，並在 encounter area 附近執行 PGO。
```

需要元件：

1. 維護每台機器人的 keyframe graph。
2. encounter 發生時加入 cross-robot edge。
3. 使用兩台機器人近期 keyframes 執行 local PGO。
4. 更新 corrected poses。
5. 重新建立並發布 corrected full map 給 Unity。

---

## 9. 建議第三階段里程碑

### Milestone 2.5：Shared retrieval database 與 cross-robot loop closure candidates

目標：

```text
在不一開始合併兩台 robot 的 live FactorGraph 的前提下，
讓 robot1 與 robot2 的 keyframe descriptors 進入同一個 shared retrieval database。

當任一 robot 產生新 keyframe 時，可以查詢 shared database，
找出另一台 robot 曾看過的相似 keyframes，
作為 cross-robot loop closure / encounter candidates。
```

這個 milestone 的核心想法是：

```text
Shared retrieval database:
  robot1 descriptors
  robot2 descriptors
  用來產生 cross-robot candidate matches

Per-robot local graphs:
  robot1 local graph
  robot2 local graph
  相遇前仍各自獨立做 local PGO

Fused global graph:
  encounter candidate 通過幾何驗證後，
  才建立 cross-robot edge 並執行 fused PGO
```

這樣可以利用 MASt3R-SLAM 原本 `RELOC` / retrieval 的精神，但避免一開始就把兩台 robot 的 keyframes 混進同一個 live `SharedKeyframes` 裡。

需要元件：

1. 每個 keyframe 都要有 `robot_id` 與 `local_keyframe_id`。
2. 每個 keyframe 的 visual descriptor / retrieval feature 要能被匯出。
3. shared retrieval database 要支援 namespaced keyframe identity。
4. 查詢結果要能區分 intra-robot match 與 cross-robot match。
5. cross-robot candidate 需要再用 MASt3R-style matching 或 pointmap alignment 做幾何驗證。
6. 驗證通過後，產生一條 cross-robot edge：

```text
robot1_kf_i <-> robot2_kf_j
```

7. 這條 edge 進入 fused graph，而不是直接塞回某一台 robot 的 local graph。

重要設計界線：

```text
可共享：
  retrieval descriptors
  keyframe image metadata
  candidate match results

暫時不共享：
  live SharedKeyframes buffer
  live FrameTracker state
  live per-robot backend FactorGraph
```

原因是現有 MASt3R-SLAM backend 假設所有 keyframes 都在同一個 map frame 裡，而且 keyframe index 是單一連續整數。如果 robot1 與 robot2 在 encounter 前尚未有 inter-map transform，就直接混用同一個 live FactorGraph，容易建立錯誤的 temporal edge，例如：

```text
robot1_kf_10 -> robot2_kf_11
```

這種 edge 在幾何上沒有意義，會讓 pose graph 被拉歪。

因此 Milestone 2.5 的推薦策略是：

```text
shared retrieval database first
cross-robot candidate detection second
geometric verification third
fused graph edge insertion last
```

它可以作為 Milestone 2 與 Milestone 3 之間的橋樑：

```text
Milestone 2:
  AprilTag / known transform based encounter edge + local PGO

Milestone 2.5:
  Shared retrieval database detects cross-robot loop closure candidates

Milestone 3:
  Fully visual encounter detection without AprilTag dependency
```

### Milestone 3：基於 visual keyframe 的 encounter detection

目標：

```text
降低對 AprilTag 的依賴，改用 keyframe retrieval 與 MASt3R-style matching 偵測共同場景。
```

需要元件：

1. 儲存每個 keyframe 的 visual descriptors。
2. 搜尋 cross-robot keyframe candidates。
3. 估計 candidate keyframes 之間的 relative transform。
4. 驗證 overlap 與 alignment quality。
5. 若通過驗證，加入 cross-robot edge。

---

## 10. 開發總結

目前開發目標是建立一套多機器人重建系統。每台機器人先獨立重建自己的環境，當機器人相遇或觀測到共同場景後，再將不同 local maps 融合成同一張 global map。

最重要的核心概念是：

```text
相遇前：
  maps 是獨立的，不能直接融合。

相遇時：
  估計 inter-map transform。

相遇後：
  將 buffered keyframes 與後續 keyframes 轉換到 shared global frame。

融合後：
  在 encounter edge 附近執行 PGO，並用 corrected global map 更新 Unity。
```

推薦第一版架構：

```text
robot1 MASt3R-SLAM process
robot2 MASt3R-SLAM process
shared multi_robot_fusion_node
Unity global visualization with incremental build + global redraw
```

這樣的設計可以保持系統模組化、容易 debug，也更接近真實的多機器人部署情境。

---

## 11. Codex codebase feasibility review

本節是根據目前 `locobot` codebase 對上述設計做的可行性評估。

### 11.1 總體判斷

整體方向是可行的，而且第一版選擇「每台機器人各自跑 MASt3R-SLAM process，再由一個 fusion node 融合」是目前最合理的路線。

原因是目前 codebase 已經有以下基礎：

1. `mast3r_slam_visual_IGBR.py` 已經能把 MASt3R-SLAM 的 keyframe pointcloud 送出。
2. `ipc_bridge_node.py` 與 `ipc_pointcloud_receiver.py` 已經提供 image input 與 pointcloud output 的 IPC/ROS bridge。
3. `auto_anchor_from_pointcloud_stretch3.py` 已經有 local `world_frame -> mast3r_map` anchoring 邏輯。
4. `pc2_to_map_stretch3.py` 已經可以把 MASt3R 點雲透過 TF 轉到 ROS world frame。
5. IGBR full-map chunking 已經很接近 multi-robot fusion 後需要的 corrected full map redraw。

所以 Milestone 1，也就是「不做 full PGO，只做 encounter transform 後的 pointcloud/keyframe fusion」，技術上可行。

但是 Milestone 2 的「真正 cross-robot PGO」目前還不能直接套用現有 MASt3R-SLAM backend，需要額外設計新的 fused graph layer。

---

### 11.2 目前 codebase 支援得最好的部分

#### A. 每台機器人獨立 pipeline

目前 `launch_mast3r_visual_ros2_igbr.sh` 已經透過環境變數支援：

```text
IPC_SOCKET
IPC_POINTCLOUD_SOCKET
MAST3R_CONFIG
MAST3R_SAVE_AS
```

這表示第一版可以很自然地開兩套 MASt3R-SLAM instance：

```text
robot1:
  IPC_SOCKET=/tmp/ipc_socket/robot1/mast3r_image.sock
  IPC_POINTCLOUD_SOCKET=/tmp/ipc_socket/robot1/mast3r_pointcloud.sock

robot2:
  IPC_SOCKET=/tmp/ipc_socket/robot2/mast3r_image.sock
  IPC_POINTCLOUD_SOCKET=/tmp/ipc_socket/robot2/mast3r_pointcloud.sock
```

需要注意的是，launch script 目前預設仍是：

```text
/tmp/ipc_socket/locobot/...
```

所以第一版需要把 quick start scripts 參數化，不能硬寫 `locobot`。

#### B. ROS topic namespace

`ipc_bridge_node.py` 和 `ipc_pointcloud_receiver.py` 都已經把 topic / socket path 做成 ROS parameters。

這很好，代表不用大改 node，只要 launch 時指定：

```text
robot1:
  image_topic:=/robot1/camera/...
  socket_path:=/tmp/ipc_socket/robot1/mast3r_image.sock
  output_topic:=/robot1/mast3r/frame_pointcloud

robot2:
  image_topic:=/robot2/camera/...
  socket_path:=/tmp/ipc_socket/robot2/mast3r_image.sock
  output_topic:=/robot2/mast3r/frame_pointcloud
```

目前缺的是一份 multi-robot launch file 或 quick_start scripts，不是底層功能。

#### C. Unity full map redraw

`mast3r_slam_visual_IGBR.py` 目前的 `publish_full_map_pointcloud()` 已經支援 chunked full map sending。

這對 multi-robot fusion 很重要，因為 encounter 後地圖會大幅修正，不適合只 append incremental keyframes。

第一版可以重用這個概念，但 fused full map 最好由 `multi_robot_fusion_node` 發布，而不是讓單一 robot 的 MASt3R node 直接負責。

---

### 11.3 目前 codebase 不足的部分

#### A. 沒有真正的 keyframe metadata topic

目前 `mast3r_slam_visual_IGBR.py` 主要輸出的是：

```text
/mast3r/frame_pointcloud
/mast3r/pointcloud_in_map
```

或 IPC binary pointcloud。

但是 multi-robot fusion 需要的不只是點雲，還需要每個 keyframe 的 metadata：

```text
robot_id
local_keyframe_id
timestamp
local pose T_map_kf
confidence threshold
is_full_map_chunk
chunk index / chunk total
source map frame
```

目前很多資訊被塞在 `PointCloud2.header.frame_id` 裡，例如 `kf_42` 或 `kf_999999_N_T`。這對 Unity demo 可以，但對 multi-robot fusion node 不夠健壯。

第一版至少應新增一個 structured metadata topic，例如：

```text
/robot1/mast3r/keyframe_metadata
/robot2/mast3r/keyframe_metadata
```

如果不想新增 custom msg，第一版可先用 `std_msgs/String` JSON。

#### B. `PointCloud2.header.frame_id` 被過度使用

目前 `ipc_pointcloud_receiver.py` 把 `kf_id` 編進：

```text
msg.header.frame_id = f"kf_{kf_id}"
```

而 `pc2_to_map_stretch3.py` 又把 TF 資訊 pack 回 frame id：

```text
kf_42|tx:...|ty:...|...
```

這對單機 Unity pipeline 是可用的 workaround，但 multi-robot 後會變脆弱：

1. frame id 本來應該表示座標 frame，不應該承載 keyframe metadata。
2. robot identity 沒有結構化欄位。
3. chunk id / keyframe id / TF info 混在同一個字串裡，不利於 fusion node 判斷。

建議第一版開始逐步改成：

```text
PointCloud2.header.frame_id = robot1/mast3r_map
metadata topic carries keyframe_id and chunk info
```

為了不破壞 Unity，短期可以保留舊 frame_id encoding，同時新增 metadata topic。

#### C. 目前沒有跨機器人的 keyframe buffer

文件提到 encounter 前 remote keyframes 要 buffer，這是必要的。

目前 codebase 中沒有這個 node。

需要新增：

```text
multi_robot_fusion_node
```

它至少要維護：

```text
robot_buffers = {
  robot1: list of KeyframeCloud,
  robot2: list of KeyframeCloud,
}

map_transforms = {
  robot1: T_global_robot1_map,
  robot2: T_global_robot2_map,
}
```

Milestone 1 不需要接 MASt3R 內部 `SharedKeyframes`，可以先只 buffer ROS `PointCloud2` 和 metadata。

#### D. 目前沒有 inter-map transform input

文件中提到 AprilTag encounter transform，但 codebase 目前還沒有對應 node。

第一版可以先做一個最簡單的接口：

```text
/multi_robot/encounter_transform
```

內容可以是：

```text
parent_map: robot1/mast3r_map
child_map: robot2/mast3r_map
transform: geometry_msgs/TransformStamped
```

如果不想做 custom msg，第一版可以用 TF：

```text
global_map -> robot1/mast3r_map
global_map -> robot2/mast3r_map
```

fusion node 只要查得到這兩條 TF，就可以開始融合。

#### E. 目前 MASt3R backend 不能直接做 multi-robot PGO

這是最重要的技術界線。

現有 `FactorGraph` 假設所有 keyframes 都在同一個 `SharedKeyframes` buffer 裡，且 index 是單一連續整數：

```text
0, 1, 2, 3, ...
```

它不理解：

```text
robot1_kf_5
robot2_kf_3
```

也不理解「兩個獨立 MASt3R process 的 keyframes」。

因此 Milestone 2 不能直接呼叫現有 `run_backend()` 來做 cross-robot PGO。要做會有兩種路：

1. 新增一個 centralized `FusedKeyframeGraph`，把 robot id + local kf id 映射成 global graph node id。
2. 或寫一個獨立的 local PGO module，只針對 fusion node 收到的 poses / pointcloud overlap 做優化。

第一版應避免動 `mast3r_slam/global_opt.py`。先完成非 PGO fusion，確認資料流、namespace、Unity redraw 都工作。

---

### 11.4 第一版建議實作範圍

建議第一版只做 Milestone 1：

```text
Two independent MASt3R-SLAM pipelines
+ namespaced IPC / ROS topics
+ fusion node buffering pointclouds
+ manually provided or AprilTag provided inter-map transform
+ fused global pointcloud publish
+ Unity global redraw
```

第一版不要做：

```text
cross-robot MASt3R feature matching
cross-robot PGO
Sim3 scale refinement
deep modification of SharedKeyframes / FactorGraph
```

這樣可以先把多機器人系統架構跑起來，避免一開始卡在 MASt3R backend 內部。

---

### 11.5 具體需要修改或新增的地方

#### 1. quick_start scripts

目前 quick start scripts 是單機導向。

需要新增類似：

```text
quick_start_multi_robot/
  robot1_ipc_bridge.sh
  robot1_mast3r_slam.sh
  robot1_ipc_receiver.sh
  robot2_ipc_bridge.sh
  robot2_mast3r_slam.sh
  robot2_ipc_receiver.sh
  multi_robot_fusion.sh
  rosbridge.sh
```

或更乾淨地寫成一份可參數化 script：

```text
run_robot_pipeline.sh robot1
run_robot_pipeline.sh robot2
```

#### 2. launch script socket namespace

`launch_mast3r_visual_ros2_igbr.sh` 已支援 env var，但 `quick_start/2_mast3r_slam.sh` 目前直接呼叫：

```text
launch_mast3r_visual_ros2_igbr.sh --viz --use-calib
```

需要讓它能傳：

```text
ROBOT_ID
IPC_SOCKET
IPC_POINTCLOUD_SOCKET
MAST3R_SAVE_AS
```

#### 3. `mast3r_slam_visual_IGBR.py`

建議新增參數：

```text
MAST3R_ROBOT_ID
MAST3R_MAP_FRAME
MAST3R_FRAME_PC_TOPIC
MAST3R_FULLMAP_TOPIC
```

ROS mode 下目前 topic 是硬寫：

```text
/mast3r/frame_pointcloud
/mast3r/pointcloud_in_map
```

multi-robot 需要改成可參數化，例如：

```text
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/pointcloud_in_map
/robot2/mast3r/frame_pointcloud
/robot2/mast3r/pointcloud_in_map
```

IPC mode 下也應讓 sender 帶出 `robot_id` metadata。

#### 4. `ipc_pointcloud_receiver.py`

需要新增或確認參數：

```text
robot_id
map_frame
metadata_topic
```

並發布 metadata：

```text
{
  "robot_id": "robot1",
  "keyframe_id": 42,
  "is_full_map": false,
  "chunk_index": null,
  "chunk_total": null,
  "pointcloud_topic": "/robot1/mast3r/frame_pointcloud",
  "map_frame": "robot1/mast3r_map"
}
```

#### 5. `pc2_to_map_stretch3.py`

短期可以繼續用於單機，但 multi-robot fusion node 最好不要依賴它把 transform 塞進 `frame_id`。

建議第一版 fusion node 自己做 pointcloud transform：

```text
robot local mast3r_map -> global_map
```

這樣流程更清楚。

#### 6. 新增 `multi_robot_fusion_node.py`

第一版核心 node。

功能：

1. 訂閱每台 robot 的 keyframe pointcloud + metadata。
2. encounter 前 buffer。
3. 查 TF 或訂閱 encounter transform。
4. transform buffered clouds into `global_map`。
5. 發布 `/multi_robot/global_pointcloud` 或 chunked `/multi_robot/pointcloud_in_map`。
6. encounter 後新 keyframes 直接轉成 global frame 後發布。

第一版可以只處理 pointcloud，不處理 MASt3R descriptors。

#### 7. Unity receiver

Unity 端目前可能依賴 `kf_42` 或 `kf_999999_N_T` frame id pattern。

multi-robot 後建議 frame id pattern 至少包含 robot：

```text
robot1_kf_42
robot2_kf_42
global_chunk_3_27
```

若 Unity 端短期不能改太多，fusion node 可以轉成 Unity 現有格式，但內部 metadata 要保留 robot identity。

---

### 11.6 技術風險排序

#### 低風險

- 兩套 IPC socket namespace。
- ROS topic namespace。
- buffer PointCloud2。
- 用已知 transform 合併兩台機器人的 pointcloud。
- chunked full-map redraw 給 Unity。

#### 中風險

- AprilTag encounter transform 的 frame chain 正確性。
- `mast3r_map` 與 robot odom/map frame 的 anchor 是否穩定。
- Unity 切換 local maps 到 global fused map 時的視覺連續性。
- 大點雲 buffer 記憶體與 bandwidth。

#### 高風險

- cross-robot PGO。
- visual keyframe matching 取代 AprilTag。
- Sim3 scale correction。
- 把兩個獨立 MASt3R `SharedKeyframes` 合成同一個 `FactorGraph`。

---

### 11.7 建議開發順序

推薦順序如下：

1. 先讓同一台機器人的 pipeline 支援 `robot_id` 與 topic/socket namespace。
2. 在同一台電腦上模擬 robot1 / robot2 兩套 pipeline，確認資料不互相污染。
3. 新增 `multi_robot_fusion_node`，先只 buffer 和 republish pointcloud。
4. 手動給定 `T_global_robot2_map`，確認 robot2 buffered clouds 可以轉到 global frame。
5. 接 AprilTag 或 TF encounter transform。
6. 做 fused full-map chunked redraw 給 Unity。
7. 再開始設計 cross-robot edge 與 local PGO。

這樣每一步都有可驗證成果，而且不需要一開始深改 MASt3R-SLAM backend。
