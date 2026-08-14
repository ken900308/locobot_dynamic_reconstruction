# LoCoBot Quick Start

這個目錄頂層只有一個公開入口：

```bash
./2_mast3r_slam.sh
```

## 常用設定

```bash
ROBOT_ID=robot1 \
ROBOT_ROSBRIDGE_HOST=192.168.0.150 \
ROBOT_ROSBRIDGE_PORT=9091 \
./2_mast3r_slam.sh
```

也可用參數覆寫：

```bash
./2_mast3r_slam.sh \
  --ip 192.168.0.150 \
  --port 9091 \
  --image-topic /locobot/camera/camera/color/image_raw/compressed \
  --camera-info-topic /locobot/camera/camera/color/camera_info \
  --tf-topic /tf \
  --tf-static-topic /locobot/tf_static_relay \
  --odom-topic /locobot/odom
```

`--odom-traj` 與 `--odom-topic` 只能擇一。使用 trajectory file 時，需先把
預設的 odom topic 清空：

```bash
MAST3R_ODOM_TOPIC= ./2_mast3r_slam.sh --odom-traj /path/to/trajectory.txt
```

正常停止請按 Ctrl-C，讓 MASt3R-SLAM 有時間儲存 final reconstruction。

## 職責邊界

LoCoBot 端不執行 backend、geometric verifier、PGO、optimized-map merger 或
ROS-TCP transfer。這些 multi-robot 元件統一由 Stretch 的 quick-start 啟動。

其餘舊腳本已移至 `legacy/`，本次不再視為可直接執行的現行流程。
