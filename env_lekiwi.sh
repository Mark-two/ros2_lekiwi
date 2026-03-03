#!/bin/bash

echo "🚀 正在为 Kang 注入具身智能环境..."

# 1. 激活你的 Conda 虚拟环境
# (如果你用的是 miniconda，或者路径不同，请自行修改下面这句 conda.sh 的路径)
source /home/kang/Documents/lerobot/.venv/bin/activate 
echo "✅ uv 环境 (lerobot) 已激活"

# 2. 刷入 ROS 2 基础环境 (假设你用的是 Humble，如果是别的版本请改掉)
source /opt/ros/humble/setup.bash
echo "✅ ROS 2 基础环境已刷入"

# 3. 刷入你的驱动工作空间
source ~/Documents/lekiwi_driver/install/setup.bash
echo "✅ Lekiwi 驱动工作空间已刷入"

# 4. 终极杀手锏：强制注入 lerobot 的 Python 路径！
# ⚠️ 极其关键：请把你真实存放 lerobot 源码库的绝对路径替换掉下面这行！
# 比如如果你的 lerobot 文件夹在 /home/kang/Documents/lerobot
export PYTHONPATH=/home/kang/Documents/lerobot/src/lerobot:$PYTHONPATH
export PYTHONPATH=/home/kang/Documents/lerobot/.venv/lib/python3.10/site-packages:$PYTHONPATH
echo "✅ lerobot 模块路径已强制挂载"

echo "🎉 全系统就绪！去让底盘跑起来吧！"