# LeKiwi 全向轮底盘 ROS 2 驱动 — 启动指南

> **硬件平台**: Jetson Orin NX + 飞特 STS3215 舵机 × 3  
> **ROS 版本**: ROS 2 Humble  
> **Python 环境**: lerobot `.venv` (uv 管理)

---

## 目录结构

```
ros2_lekiwi/
├── env_lekiwi.sh                 # 一键环境注入脚本 (必须 source)
├── ros2_cmd.sh                   # 调试命令速查手册
├── lekiwi_bring_up.launch.py     # Launch 文件
└── src/lekiwi_driver/            # 驱动包源码
    └── lekiwi_driver/
        ├── __init__.py
        └── driver_node.py        # 底盘驱动节点
```

---

## 1. 前置条件

| 项目 | 要求 |
|------|------|
| ROS 2 Humble | `source /opt/ros/humble/setup.bash` 可执行 |
| lerobot 源码 | 位于 `/home/kang/Documents/lerobot/` |
| lerobot venv | `/home/kang/Documents/lerobot/.venv/` 已安装依赖 |
| 串口设备 | `/dev/lekiwi_driver` 存在 (udev 规则已配置) |
| teleop 键盘包 | `sudo apt install ros-humble-teleop-twist-keyboard` |

---

## 2. 编译工作空间 (首次 / 代码修改后)

```bash
cd ~/Documents/ros2_lekiwi

# 刷入 ROS 2 环境
source /opt/ros/humble/setup.bash

# 编译 (如果有残留问题，先 rm -rf build/ install/ log/)
colcon build
```

> **踩坑记录**: 如果 `colcon build` 报 `setup.py: No such file or directory`，执行 `rm -rf build/ install/ log/` 后重新编译。

---

## 3. 启动底盘 (每次开机后)

### 步骤一：注入环境

```bash
cd ~/Documents/ros2_lekiwi

# ⚠️ 必须用 source，不能用 ./env_lekiwi.sh (子shell中环境变量会丢失)
source ./env_lekiwi.sh
```

看到以下输出即为成功：

```
🚀 正在为 Kang 注入具身智能环境...
✅ uv 环境 (lerobot) 已激活
✅ ROS 2 基础环境已刷入
✅ Lekiwi 驱动工作空间已刷入
✅ lerobot 模块路径已强制挂载
🎉 全系统就绪！去让底盘跑起来吧！
```

### 步骤二：启动驱动节点

```bash
ros2 run lekiwi_driver base_driver
```

正常启动后会看到电机逐个初始化的日志。

### 步骤三：键盘遥控 (新开终端)

```bash
# 新终端也需要先注入环境！
cd ~/Documents/ros2_lekiwi
source ./env_lekiwi.sh

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按键说明：
- `i` 前进 / `k` 刹车 / `,` 后退
- `j` 左转 / `l` 右转
- `J` (Shift+j) 左平移 / `L` (Shift+l) 右平移 (全向轮专属)

---

## 4. 常用调试命令

```bash
# 查看里程计数据 (只看位姿，不看协方差矩阵)
ros2 topic echo /odom --field pose.pose

# 查看完整里程计
ros2 topic echo /odom

# 手动发送速度指令 (前进 0.1m/s，10Hz)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 紧急刹车 (单次全零)
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Foxglove 可视化桥接 (局域网浏览器打开 ws://<Jetson-IP>:8765)
ros2 run foxglove_bridge foxglove_bridge
```

---

## 5. 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `Package 'lekiwi_driver' not found` | 没有 source 环境，或用了 `./` 执行脚本 | `source ./env_lekiwi.sh` |
| `No module named 'lerobot'` | PYTHONPATH 未正确注入 | 确认 `source ./env_lekiwi.sh` 已执行，检查 `env_lekiwi.sh` 中路径是否为 `lerobot/src`（不是 `lerobot/src/lerobot`） |
| `colcon build` 报 setup.py 找不到 | build 目录残留脏数据 | `rm -rf build/ install/ log/ && colcon build` |
| 电机无反应 | 串口设备名不对 | 检查 `ls /dev/lekiwi_driver`，确认 udev 规则 |
| 电机初始化超时告警 | 飞特固件 EEPROM 写入偶发慢 | 通常可忽略，节点会继续运行 |

---

## 6. 快速启动 Cheatsheet

```bash
# === 终端 1: 驱动 ===
cd ~/Documents/ros2_lekiwi
source ./env_lekiwi.sh
ros2 run lekiwi_driver base_driver

# === 终端 2: 键盘遥控 ===
cd ~/Documents/ros2_lekiwi
source ./env_lekiwi.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
