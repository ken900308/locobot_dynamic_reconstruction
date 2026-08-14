# MASt3R-SLAM（LoCoBot 現行整合）

這個目錄只保留目前由 `thor/quick_start/2_mast3r_slam.sh` 啟動的
LoCoBot MASt3R-SLAM 路徑，以及其執行期與建置期依賴。

## Active 啟動鏈

```text
quick_start/2_mast3r_slam.sh
  └─ launch_mast3r_visual_ros2_igbr.sh
      └─ mast3r_slam_visual_IGBR.py
          ├─ main_mast3r.py
          ├─ mast3r_slam/
          ├─ config/{base,calib,eval_no_calib,intrinsics}.yaml
          ├─ resources/programs/*.glsl
          ├─ thirdparty/{mast3r,in3d,eigen}
          └─ mast3r_slam_backends*.so
```

日常操作應從 quick-start 入口執行，不要直接執行 Python entrypoint，否則
robot1 namespace、topics、rosbridge 與 shutdown/save 設定不會完整套用。

## Active 內容

- `launch_mast3r_visual_ros2_igbr.sh`：唯一受支援的 launcher。
- `mast3r_slam_visual_IGBR.py`：LoCoBot ROS/rosbridge 整合。
- `main_mast3r.py`、`mast3r_slam/`：SLAM backend 與演算法 modules。
- `config/`：現行 calib/no-calib 與 LoCoBot camera intrinsics。
- `resources/`、`thirdparty/`：visualization、模型與編譯依賴。
- `setup.py`、`pyproject.toml`：CUDA/C++ backend 建置設定。
- 授權及 dependency metadata。

不在現行啟動鏈上的檔案已移到 `legacy/`。詳見
`legacy/README.md`。
