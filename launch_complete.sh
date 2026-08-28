#!/bin/bash
# QD_Vision2026 完整系统启动脚本

echo "=== QD Vision 2026 完整系统启动 ==="
echo "时间: $(date)"
echo ""

# 1. 进入工作空间
cd ~/QD_Vision2026

# 2. 设置环境变量
echo "设置环境变量..."
export LD_LIBRARY_PATH=/home/scurm/QD_Vision2026/.pixi/envs/default/lib:/home/scurm/QD_Vision2026/src/rm_utils/hikSDK/lib/amd64:$LD_LIBRARY_PATH
export LD_PRELOAD=/home/scurm/QD_Vision2026/.pixi/envs/default/lib/libstdc++.so.6
export PATH=/home/scurm/QD_Vision2026/.pixi/envs/default/bin:$PATH

# 3. 加载 ROS 环境
echo "加载 ROS 环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. 显示环境信息
echo "----------------------------------------"
echo "LD_PRELOAD: $LD_PRELOAD"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "----------------------------------------"

# 5. 启动系统
echo "启动系统..."
ros2 launch rm_bringup bringup_SingleProcess.launch.py
