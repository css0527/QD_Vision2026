#!/bin/bash
# QD_Vision2026 完整系统启动脚本

echo "=== QD Vision 2026 完整系统启动 ==="
echo "时间: $(date)"
echo ""

# 1. 进入工作空间（脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 2. 清理残留进程，避免相机句柄和 ROS 组件容器被旧进程占用
echo "清理残留进程..."
for pattern in "component_container_mt" "ros2 launch rm_bringup bringup_SingleProcess.launch.py"; do
    pids=$(ps -eo pid,cmd | awk -v pat="$pattern" '$0 ~ pat {print $1}')
    if [ -n "$pids" ]; then
        echo "终止: $pattern -> $pids"
        kill $pids 2>/dev/null || true
        sleep 0.5
        kill -9 $pids 2>/dev/null || true
    fi
done

# 3. 设置环境变量
echo "设置环境变量..."
export LD_LIBRARY_PATH="$SCRIPT_DIR/.pixi/envs/default/lib:$SCRIPT_DIR/src/rm_utils/hikSDK/lib/amd64:$LD_LIBRARY_PATH"
export LD_PRELOAD="$SCRIPT_DIR/.pixi/envs/default/lib/libstdc++.so.6"
export PATH="$SCRIPT_DIR/.pixi/envs/default/bin:$PATH"
# 让 OpenVINO GPU 插件能找到系统 OpenCL ICD（核显加速）
export OCL_ICD_VENDORS="/etc/OpenCL/vendors"

# 4. 加载 ROS 环境
echo "加载 ROS 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 5. 显示环境信息
echo "----------------------------------------"
echo "LD_PRELOAD: $LD_PRELOAD"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "----------------------------------------"

# 6. 启动系统
echo "启动系统..."
ros2 launch rm_bringup bringup_SingleProcess.launch.py
