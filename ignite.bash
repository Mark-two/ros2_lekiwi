#!/bin/bash

echo "=================================================="
echo "⚙️ [Lekiwi] 正在进行发射前置准备..."
echo "=================================================="

# 1. 注入灵魂环境
echo "🧬 正在注入 LeRobot 与 ROS 2 运行环境..."
source ~/Documents/ros2_lekiwi/env_lekiwi.sh
source ~/Documents/ros2_lekiwi/install/setup.bash

# 2. 终极点火
echo "🚀 引擎点火！全系统节点拉起！(按 Ctrl+C 即可完美退出)"
echo "=================================================="
ros2 launch /home/kang/Documents/ros2_lekiwi/lekiwi_bring_up.launch.py