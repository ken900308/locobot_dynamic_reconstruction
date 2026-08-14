# LoCoBot MASt3R-SLAM Client

LoCoBot 端目前只負責執行自己的 MASt3R-SLAM，公開入口只有
`thor/quick_start/2_mast3r_slam.sh`。

Multi-robot backend、geometric verification、PGO、optimized map 與 ROS-TCP
transfer 全部由 `stretch3_jazzy` 端啟動；不要在 LoCoBot 端重複啟動。

## 啟動

在 LoCoBot 的 Thor／MASt3R container 內執行：

```bash
cd /workspace/thor/quick_start
./2_mast3r_slam.sh
```

目前腳本預設值：

```text
ROBOT_ID=robot1
ROBOT_ROSBRIDGE_HOST=192.168.0.150
ROBOT_ROSBRIDGE_PORT=9091
ROBOT_IMAGE_TOPIC=/locobot/camera/camera/color/image_raw/compressed
ROBOT_CAMERA_INFO_TOPIC=/locobot/camera/camera/color/camera_info
ROBOT_TF_TOPIC=/tf
ROBOT_TF_STATIC_TOPIC=/locobot/tf_static_relay
MAST3R_ODOM_TOPIC=/locobot/odom
```

實際部署時可用環境變數或 command-line 參數覆寫。完整說明請看
[`thor/quick_start/README.md`](thor/quick_start/README.md)。

## LoCoBot 輸出

```text
/robot1/tf
/robot1/tf_static
/robot1/mast3r/frame_pointcloud
/robot1/mast3r/pointcloud_in_map
/robot1/mast3r/pointcloud_in_mast3r_map
/robot1/mast3r/keyframe_metadata
/robot1/mast3r/keyframe_cloud_local
/robot1/mast3r/keyframe_image
```

Stretch 端的 8/9/10 與 transfer 流程會訂閱其中需要的 robot1 topics。

## 目錄

```text
locobot_jazzy/
├── docs/                       # 封存文件
└── thor/
    ├── MASt3R-SLAM/            # 現行 LoCoBot SLAM 實作
    │   └── legacy/             # 舊 entrypoints/evaluation/backups
    └── quick_start/
        ├── 2_mast3r_slam.sh    # 唯一公開入口
        └── legacy/             # LoCoBot 不再啟動的舊流程
```

Build、install、log、checkpoint、rosbag 與 cache 是本機產物，不屬於 active
原始碼分類。

## 舊文件

原本描述 2/4/5 流程的 README 已移到
`docs/legacy_pipeline_README.md`，只供歷史追溯。
