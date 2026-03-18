#!/bin/bash
# ==============================================================================
# 🤖 全向轮底盘 (Jetson Orin NX + 飞特舵机) ROS 2 调试速查手册
# ==============================================================================

echo "欢迎使用底盘调试速查表！请打开脚本文件，复制需要的命令到新终端运行。"

# ------------------------------------------------------------------------------
# 1. 话题直发测试 (底层电机与逆解验证 - 需悬空小车测试！)
# ------------------------------------------------------------------------------

# 持续发送前进指令 (10Hz 频率，0.1m/s)
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 持续发送横移指令 (螃蟹步，0.1m/s)
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 持续发送自转指令 (原地打转，0.5rad/s)
# ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# 单次急刹车指令 (强制全零停机，无 DDS 握手延迟测试)
# ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


# ------------------------------------------------------------------------------
# 2. 键盘遥控操作 (常驻节点的丝滑控制)
# ------------------------------------------------------------------------------

# 启动标准键盘遥控器 
# (按 i 前进，k 刹车，j/l 旋转。隐藏技巧：Shift + J/L 触发全向轮专属的 Y 轴平移！)
# ros2 run teleop_twist_keyboard teleop_twist_keyboard


# ------------------------------------------------------------------------------
# 3. 里程计与状态查验 (抓虫与正解验证)
# ------------------------------------------------------------------------------

# 监听完整的里程计数据 (包含满屏的协方差矩阵)
# ros2 topic echo /odom

# 狙击特定字段 (强烈推荐！只看干净清爽的 XYZ 坐标和四元数姿态)
# ros2 topic echo /odom --field pose.pose


# ------------------------------------------------------------------------------
# 4. 节点运行与现代可视化 (生态打通)
# ------------------------------------------------------------------------------

# 启动底盘底层驱动节点 (确保当前终端已 source 环境)
# ros2 run lekiwi_driver base_driver

# 启动 Foxglove 局域网桥接器 (默认监听 8765 端口)
# (启动后，在本地电脑打开 Foxglove 客户端 -> Open Connection -> Foxglove WebSocket -> ws://<Jetson-IP>:8765)
# ros2 run foxglove_bridge foxglove_bridge


# ------------------------------------------------------------------------------
# 5. YOLO + RealSense 视觉检测 (一键启动相机驱动 + TensorRT 推理)
# ------------------------------------------------------------------------------

# source install/setup.bash
# ros2 launch lekiwi_vision yolo_realsense.launch.py


echo "======================================================"
echo "🚀 启动 D435i 视觉惯性里程计 (VIO) 融合建图系统"
echo "======================================================"

# 1. 声明工作空间环境变量 (极其重要)
source ~/ros2_ws/install/setup.bash

# 2. 设置进程清理陷阱 (Trap)：捕捉 Ctrl+C 信号，实现一键安全退出
trap 'echo -e "\n🛑 收到退出信号，正在安全关闭所有节点..."; kill 0' SIGINT

# 3. 启动相机硬件驱动
echo "[1/3] 正在拉起 RealSense 底层驱动 (等待 5 秒硬件复位)..."
ros2 launch realsense2_camera rs_launch.py \
    initial_reset:=true \
    rgb_camera.color_profile:=640x480x15 \
    depth_module.depth_profile:=640x480x15 \
    align_depth.enable:=true \
    enable_gyro:=true \
    enable_accel:=true \
    unite_imu_method:=2 &

# 等待相机硬件复位并加载完毕
sleep 5

# 4. 启动 IMU 滤波器
echo "[2/3] 正在拉起 Madgwick IMU 滤波器..."
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
    --ros-args \
    -p use_mag:=false \
    -p publish_tf:=false \
    -r /imu/data_raw:=/camera/camera/imu \
    -r /imu/data:=/imu/filtered &

# 等待滤波器就绪
sleep 2

# 5. 启动 RTAB-Map 核心算法
echo "[3/3] 正在拉起 RTAB-Map 融合建图算法 (UI已关闭)..."
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start --Optimizer/GravitySigma 0.3" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    imu_topic:=/imu/filtered \
    frame_id:=camera_link \
    approx_sync:=true \
    wait_imu_to_init:=true \
    sync_queue_size:=30 \
    topic_queue_size:=30 \
    qos:=1 \
    rviz:=false \
    rtabmap_viz:=false &

echo "======================================================"
echo "✅ 所有节点已在后台全速运行！"
echo "🎮 请打开 Foxglove 查看实时的彩色 3D 点云状态。"
echo "🛑 想要结束建图时，请在此终端按下 [Ctrl + C] 即可一键退出。"
echo "======================================================"

# 挂起主脚本，等待用户按 Ctrl+C，同时持续输出后台子进程的日志
wait