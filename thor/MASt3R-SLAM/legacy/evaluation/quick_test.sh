#!/bin/bash
#
# 快速模組測試腳本
# 直接在 terminal 運行來檢查關鍵依賴
#

echo "🔍 快速依賴檢查"
echo "================"

# 設置 Python 路徑
export PYTHONPATH="/workspace/MASt3R-SLAM/thirdparty/mast3r:/workspace/MASt3R-SLAM:$PYTHONPATH"

echo "📍 Python 版本:"
python3 --version

echo ""
echo "🔧 檢查關鍵模組:"

# 測試 lietorch (最關鍵的缺失模組)
echo -n "lietorch: "
python3 -c "import lietorch; print('✅ OK')" 2>/dev/null || echo "❌ MISSING"

# 測試 PyTorch
echo -n "torch: "
python3 -c "import torch; print('✅ OK - v' + torch.__version__)" 2>/dev/null || echo "❌ MISSING"

# 測試 torchvision
echo -n "torchvision: "
python3 -c "import torchvision; print('✅ OK - v' + torchvision.__version__)" 2>/dev/null || echo "❌ MISSING"

# 測試 numpy
echo -n "numpy: "
python3 -c "import numpy; print('✅ OK - v' + numpy.__version__)" 2>/dev/null || echo "❌ MISSING"

# 測試 OpenCV
echo -n "opencv-python: "
python3 -c "import cv2; print('✅ OK - v' + cv2.__version__)" 2>/dev/null || echo "❌ MISSING"

# 測試 ROS2
echo -n "rclpy: "
python3 -c "import rclpy; print('✅ OK')" 2>/dev/null || echo "❌ MISSING"

# 測試 MASt3R 模組
echo -n "mast3r: "
python3 -c "import mast3r; print('✅ OK')" 2>/dev/null || echo "❌ MISSING"

echo ""
echo "🚀 CUDA 支援:"
python3 -c "
import torch
if torch.cuda.is_available():
    print(f'✅ CUDA 可用 - v{torch.version.cuda}')
    print(f'   GPU 數量: {torch.cuda.device_count()}')
    for i in range(torch.cuda.device_count()):
        print(f'   Device {i}: {torch.cuda.get_device_name(i)}')
else:
    print('❌ CUDA 不可用')
" 2>/dev/null || echo "❌ 無法檢查 CUDA"

echo ""
echo "📁 關鍵檔案:"
files=(
    "/workspace/MASt3R-SLAM/main_mast3r.py"
    "/workspace/MASt3R-SLAM/thirdparty/mast3r/mast3r/model.py"
    "/workspace/MASt3R-SLAM/mast3r_slam/__init__.py"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file - 不存在"
    fi
done

echo ""
echo "💡 如果 lietorch 缺失，請運行:"
echo "   pip install lietorch"
echo ""
echo "🔧 如果其他模組缺失，請運行:"
echo "   pip install torch torchvision numpy opencv-python matplotlib scipy tqdm"