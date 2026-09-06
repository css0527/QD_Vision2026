#!/bin/bash
# QD_Vision2026 完整启动脚本

echo "=== QD Vision 2026 启动脚本 ==="

# 1. 进入工作空间（脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 2. 设置环境变量
echo "设置环境变量..."
export LD_LIBRARY_PATH="$SCRIPT_DIR/.pixi/envs/default/lib:$SCRIPT_DIR/src/rm_utils/hikSDK/lib/amd64:$LD_LIBRARY_PATH"
export LD_PRELOAD="$SCRIPT_DIR/.pixi/envs/default/lib/libstdc++.so.6"
export PATH="$SCRIPT_DIR/.pixi/envs/default/bin:$PATH"
# 让 OpenVINO GPU 插件能找到系统 OpenCL ICD（核显加速）
export OCL_ICD_VENDORS="/etc/OpenCL/vendors"

# 3. 确保相机权限
echo "检查相机权限..."
sudo chmod 666 /dev/bus/usb/*/* 2>/dev/null || echo "无法修改USB设备权限，请手动执行: sudo chmod 666 /dev/bus/usb/*/*"

# 4. 加载 ROS 环境
echo "加载 ROS 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 5. 显示当前环境
echo "当前环境信息:"
echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "LD_PRELOAD: $LD_PRELOAD"

# 6. 启动系统
echo "启动系统..."
echo "----------------------------------------"
ros2 launch rm_bringup bringup_SingleProcess.launch.py
