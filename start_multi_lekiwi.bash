#!/bin/bash

echo "🚀 [Lekiwi] 正在开启多终端分离调试模式 (真机硬件已接入)..."

# 统一的环境变量加载命令 (每个终端都会先无脑执行这个)
SETUP_CMD="source ~/Documents/ros2_lekiwi/install/setup.bash"

# ==============================================================================
# 巧妙处理长串指令：把提示信息写进临时文件
# ==============================================================================
cat << 'EOF' > /tmp/lekiwi_fire_hint.txt

==================================================
🚀 亢，发射台就绪，真机硬件环境已加载！

系统将在 2 秒后自动向机甲发送贴地抓取指令 (Z=0.03)...
⚠️ 请确保机械臂运动范围内没有障碍物！
如果需要手动复测，请随时复制下方指令并回车执行：
ros2 topic pub -1 /lekiwi_vision/target_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'base_link'}, point: {x: 0.2, y: 0.0, z: -0.12}}"
==================================================

EOF

# 1. 骨架引擎
echo "👉 正在打开 骨架引擎 终端..."
gnome-terminal --tab --title="🦴 骨架引擎" -- bash -c "$SETUP_CMD; ros2 run robot_state_publisher robot_state_publisher /home/kang/Documents/ros2_lekiwi/src/lekiwi_vision/urdf/so101_arm.urdf; exec bash"

sleep 1

# 2. RViz2 
echo "👉 正在打开 RViz2 终端..."
gnome-terminal --tab --title="👁️ RViz2" -- bash -c "$SETUP_CMD; ros2 run rviz2 rviz2 -d ~/Documents/ros2_lekiwi/lekiwi.rviz; exec bash"

sleep 1 

# 2.5 强制接骨 (真机测试时，这针初始角度发完后，真机会立刻接管 TF 树)
echo "👉 正在发送初始关节角度 (强制接骨)..."
gnome-terminal --tab --title="💉 接骨医生" -- bash -c "$SETUP_CMD; echo '正在打通 TF 树，唤醒机甲...'; ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState \"{header: {stamp: 'now', frame_id: 'base_link'}, name: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'], position: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]}\"; exec bash"

# 3. 大脑节点 (逆解与状态机)
echo "👉 正在打开 Pink 逆解大脑 终端..."
gnome-terminal --tab --title="🧠 大脑节点" -- bash -c "$SETUP_CMD; ros2 run lekiwi_driver lekiwi_brain_node; exec bash"

# ==============================================================================
# ⚠️ 进阶物理/视觉节点 
# ==============================================================================

# 4. 真实机械臂底层驱动 (串口通信舵机) - 💥 已解封并注入特有环境！
echo "👉 正在打开 真实机械臂驱动 终端..."
gnome-terminal --tab --title="🦾 真实机械臂" -- bash -c "$SETUP_CMD; source /home/kang/Documents/ros2_lekiwi/env_lekiwi.sh; ros2 run lekiwi_driver lekiwi_arm_driver; exec bash"

# 5. YOLO TensorRT 视觉推理节点 (继续保持注释，等调完机械臂再开)
# echo "👉 正在打开 YOLO 视觉推理 终端..."
# gnome-terminal --tab --title="👀 YOLO视觉" -- bash -c "$SETUP_CMD; ros2 run lekiwi_vision yolo_trt_node; exec bash"

# ==============================================================================

# 6. 数据监听哨 
echo "👉 正在打开 关节数据监听 终端..."
gnome-terminal --tab --title="📈 关节数据" -- bash -c "$SETUP_CMD; echo '=================================================='; echo '📈 正在死死盯住 /joint_states 频道...'; echo '=================================================='; ros2 topic echo /joint_states; exec bash"

sleep 1

# 7. 发射台 
echo "👉 正在打开 发射台 终端..."
gnome-terminal --tab --title="🎯 发射台" -- bash -c "$SETUP_CMD; cat /tmp/lekiwi_fire_hint.txt; sleep 2; ros2 topic pub -1 /lekiwi_vision/target_point geometry_msgs/msg/PointStamped \"{header: {frame_id: 'base_link'}, point: {x: 0.2, y: 0.0, z: -0.12}}}\"; exec bash"

echo "=================================================="
echo "✅ 所有激活的节点已在新的标签页中启动！"
echo "✅ 真机驱动已上线！"
echo "=================================================="