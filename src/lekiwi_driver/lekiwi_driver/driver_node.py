#!/usr/bin/env python3
# 指定使用系统中的 python3 解释器运行此脚本
import sys
# 导入 sys，用于操作 Python 运行时路径
from pathlib import Path
# 导入 Path，方便进行跨平台路径拼接与判断

try:
    # 尝试导入 ROS2 相关 Python 包
    import rclpy
    # 导入 rclpy 主模块
    from rclpy.node import Node
    # 导入 ROS2 节点基类
    from geometry_msgs.msg import Twist
    # 导入速度指令消息类型 Twist
    from nav_msgs.msg import Odometry
    # 导入里程计消息类型 Odometry
    from geometry_msgs.msg import TransformStamped, Quaternion
    # 导入 TF 变换与四元数消息类型
    from tf2_ros import TransformBroadcaster
    # 导入 TF 广播器，用于发布 odom->base_link 变换
except ModuleNotFoundError as e:
    # 如果缺少 ROS2 依赖模块，抛出更友好的错误提示
    raise ModuleNotFoundError(
        "缺少 ROS2 Python 依赖（rclpy/geometry_msgs/nav_msgs/tf2_ros）。\n"
        "请在 ROS2 环境中运行，例如先 source /opt/ros/humble/setup.bash，"
        "再 source 工作区 install/setup.bash，然后用 `ros2 run lekiwi_driver base_driver` 启动。"
    ) from e
    # 保留原始异常链，便于定位根因
import math
# 导入数学库，用于三角函数与开方计算
import time
# 导入时间库，用于初始化过程中的短暂延时

# 引入你的电机库 (假设你已经装好了 lerobot 环境)
# 如果之前是在虚拟环境 .venv 里，记得在运行这个节点前 source .venv/bin/activate
try:
    # 先按标准安装路径尝试导入 lerobot 电机总线
    from lerobot.motors.feetech import FeetechMotorsBus
    # 导入飞特舵机总线封装
    from lerobot.motors.motors_bus import Motor, MotorNormMode
    # 导入电机对象与归一化模式枚举
except ImportError:
    # 如果标准导入失败，尝试兼容本地源码开发路径
    # 兼容本地源码布局：/home/kang/Documents/lerobot/src
    candidate_src = Path(__file__).resolve().parents[4] / "lerobot" / "src"
    # 推导可能的本地 lerobot/src 路径
    if candidate_src.exists() and str(candidate_src) not in sys.path:
        # 如果路径存在且尚未加入模块搜索路径，则插入到最前
        sys.path.insert(0, str(candidate_src))
        # 将本地源码路径加入 sys.path 以支持 import

    try:
        # 再次尝试导入 lerobot 相关模块
        from lerobot.motors.feetech import FeetechMotorsBus
        # 导入飞特舵机总线封装
        from lerobot.motors.motors_bus import Motor, MotorNormMode
        # 导入电机对象与归一化模式枚举
    except ImportError as second_error:
        # 若依旧失败，进入更细化的错误处理
        missing = getattr(second_error, "name", None)
        # 读取缺失模块名（若解释器有提供）
        if missing == "lerobot":
            # 若是 lerobot 本体缺失，给出专门提示
            raise ImportError(
                "找不到 `lerobot` 库。请先安装/激活包含 lerobot 的 Python 环境，"
                "或确认本地路径 `/home/kang/Documents/lerobot/src` 可访问。"
            ) from second_error
            # 抛出带说明的 ImportError，并保留异常链

        raise ImportError(
            "导入 `lerobot` 失败，缺少依赖模块: "
            f"{missing or str(second_error)}。"
            "请在运行 `ros2 run` 的同一 Python 环境中安装依赖（常见: pyserial/deepdiff/tqdm）。"
        ) from second_error
        # 抛出通用依赖缺失提示，指导在同一环境中安装依赖

# --- 机器人物理参数 (需要你用尺子量一下修改！) ---
WHEEL_RADIUS = 0.05   # 轮子半径 (米)
ROBOT_RADIUS = 0.15   # 机器人半径 (米, 轮子触地点到中心的距离)
GEAR_RATIO = 1.0      # 如果是直驱就是1.0
# 电机 ID 定义 (按逆时针顺序: 左前, 左后, 右? 需根据实际 ID 修改)
MOTOR_IDS = [1, 2, 3] 
# 三个驱动轮对应的电机 ID 列表

class LeKiwiDriver(Node):
    # LeKiwi 底盘驱动节点类，负责接收 cmd_vel、发电机速度与里程计
    def __init__(self):
        # 构造函数，初始化 ROS 通信、电机与状态变量
        super().__init__('lekiwi_base_driver')# 初始化父类 Node，并设置节点名
        
        # 1. 初始化 ROS 通讯
        # 订阅速度指令
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # 参数说明: msg_type=Twist(消息类型), topic='cmd_vel'(话题名), callback=self.cmd_vel_callback(回调函数), qos_profile=10(队列深度)
        # 创建 cmd_vel 订阅者，收到速度指令时回调 cmd_vel_callback
        # 发布里程计
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        # 参数说明: msg_type=Odometry(消息类型), topic='odom'(话题名), qos_profile=10(队列深度)
        # 创建 odom 发布者，用于发布里程计信息
        # TF 广播器 (SLAM 必须)
        self.tf_broadcaster = TransformBroadcaster(self)
        # 参数说明: node=self(当前节点实例，广播器依附该节点进行通信)
        # 创建 TF 广播器，持续发布坐标变换
        
        # 定时器: 50Hz (每 0.02秒跑一次控制循环)
        self.create_timer(0.02, self.control_loop)
        # 参数说明: timer_period_sec=0.02(触发周期，单位秒), callback=self.control_loop(定时执行函数)
        # 创建 50Hz 定时器，周期执行控制循环

        # 2. 初始化电机 (使用你的修复逻辑)
        self.get_logger().info("正在连接电机...")

        # 打印日志，提示开始连接电机
        self.init_motors()
        # 执行电机初始化流程
        
        # 状态变量
        self.x = 0.0
        # 机器人在 odom 坐标系下的 x 位置
        self.y = 0.0
        # 机器人在 odom 坐标系下的 y 位置
        self.th = 0.0
        # 机器人航向角（yaw）
        self.last_time = self.get_clock().now()
        # 记录上一帧时间，用于计算 dt
        
        # 目标速度
        self.target_vx = 0.0
        # 目标线速度 x 分量（机体坐标系）
        self.target_vy = 0.0
        # 目标线速度 y 分量（机体坐标系）
        self.target_vth = 0.0
        # 目标角速度 z 分量（yaw 角速度）

    def init_motors(self):
        """带有防超时装甲的初始化逻辑"""
        # 初始化电机总线与电机参数，尽量规避固件偶发超时问题
        motors_config = {}
        # 创建电机配置字典，键为逻辑名，值为 Motor 对象
        for mid in MOTOR_IDS:
            # 遍历所有电机 ID 并构造配置
            motors_config[f"m{mid}"] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
            # 以 m{id} 命名电机，型号 sts3215，归一化范围设为 -100~100
        
        # 显式指定波特率，防止默认值抽风
        self.bus = FeetechMotorsBus(port='/dev/lekiwi_driver', motors=motors_config)
        # 创建电机总线对象，串口为 /dev/lekiwi_driver
        self.bus.connect()
        # 连接串口总线
        
        # 批量初始化
        for name in motors_config:
            # 遍历每个电机进行初始化配置
            mid = motors_config[name].id
            # 读取当前电机的物理 ID
            self.get_logger().info(f"正在配置电机 ID: {mid} ...")
            # 打印当前配置进度
            
            # 1. 解锁并切换模式 (RAM 操作)
            self.bus.write("Lock", name, 0)
            # 解锁寄存器写保护
            self.bus.write("Torque_Enable", name, 0)
            # 关闭扭矩，确保模式切换安全
            self.bus.write("Operating_Mode", name, 1) # Mode 1: 连续旋转模式
            # 切换到模式1（连续旋转）
            time.sleep(0.02) # ⚠️ 关键延时：给模式切换一点反应时间
            # 等待电机内部状态稳定
            
            # 2. 修复地址 46 (加 try 防止个别固件不支持报错)
            try: 
                # 尝试写入地址46做兼容修复
                self.bus._write(46, 2, mid, 3400) 
                # 向地址46写入 2 字节值 3400
                time.sleep(0.01)
                # 短暂等待写入完成
            except Exception as e: 
                # 某些固件不支持该地址，记录警告并继续
                self.get_logger().warn(f"电机 {mid} 地址 46 写入忽略: {e}")
                # 打印可忽略告警

            self.bus.write("Acceleration", name, 50)
            # 设置加速度参数，减小启动冲击
            
            # 3. 清除软限位 (EEPROM 操作，最容易引发超时无响应)
            try:
                # 写入 EEPROM 清除最小限位
                self.bus._write(9, 2, mid, 0) 
                # 地址9写0，取消一侧软限位
                time.sleep(0.01) # ⚠️ 关键延时：等待闪存写入
                # 等待 EEPROM 落盘
                self.bus._write(11, 2, mid, 0)
                # 地址11写0，取消另一侧软限位
                time.sleep(0.01)
                # 再次等待 EEPROM 落盘
            except Exception as e:
                # EEPROM 写入超时在现场常见，记录后继续
                self.get_logger().warn(f"电机 {mid} 清理限位时超时(通常可忽略): {e}")
                # 打印清理限位告警

            # 4. 重新上锁并开启扭矩
            self.bus.write("Lock", name, 1)
            # 重新开启写保护
            self.bus.write("Torque_Enable", name, 1)
            # 开启扭矩，准备运行
            
        self.get_logger().info("✅ 所有电机初始化完成 (Mode 1 + Addr 46 Fix + 稳健延时)")
        # 打印初始化完成提示

    def cmd_vel_callback(self, msg):
        """收到键盘/导航的速度指令"""
        # 将外部输入速度保存为当前目标速度
        self.target_vx = msg.linear.x
        # 保存 x 方向线速度
        self.target_vy = msg.linear.y
        # 保存 y 方向线速度
        self.target_vth = msg.angular.z
        # 保存 z 方向角速度

    def control_loop(self):
        # 周期性控制循环：计算轮速、下发电机、发布里程计与 TF
        # --- A. 逆运动学 (Inverse Kinematics): 身体速度 -> 轮子速度 ---
        # 简化的三轮全向轮公式 (120度分布)
        # v1 (左前), v2 (右前), v3 (后) - 需根据你的实际安装调整符号
        vx, vy, vth = self.target_vx, self.target_vy, self.target_vth
        # 读取当前目标机体速度
        
        # 这是一个常见的全向轮矩阵形式，如果方向不对，试着交换符号
        w1_speed_m_s = -0.5 * vx - (math.sqrt(3)/2) * vy + ROBOT_RADIUS * vth
        # 计算电机1的线速度（m/s）
        w2_speed_m_s = -0.5 * vx + (math.sqrt(3)/2) * vy + ROBOT_RADIUS * vth
        # 计算电机2的线速度（m/s）
        w3_speed_m_s =  vx + ROBOT_RADIUS * vth
        # 计算电机3的线速度（m/s）

        # 转换 m/s -> STS3215 原始单位
        # 修正：将 m/s 精准映射到飞特 ST-3215 原始速度单位 (步/秒)
        # 公式: (1 / (2 * pi * R)) * 4096
        K_SPEED = 13038  # 绝对不要再用 3000 了！
        # 速度比例系数：将 m/s 映射到舵机原始速度单位
        
        raw_v1 = w1_speed_m_s * K_SPEED
        # 电机1原始目标速度
        raw_v2 = w2_speed_m_s * K_SPEED
        # 电机2原始目标速度
        raw_v3 = w3_speed_m_s * K_SPEED
        # 电机3原始目标速度

        # =================================================================
        # 🛡️ 终极防御：全向轮等比例防抖限速 (Desaturation)
        # =================================================================
        MAX_SPEED_TICKS = 2400.0  # 飞特电机的物理天花板安全阈值
        
        # 找出当前指令中，转得最快的那个轮子的绝对值
        max_target = max(abs(raw_v1), abs(raw_v2), abs(raw_v3))
        
        # 如果最快的轮子撞到了物理限速墙，触发所有轮子等比例压缩
        if max_target > MAX_SPEED_TICKS:
            scale_factor = MAX_SPEED_TICKS / max_target
            raw_v1 *= scale_factor
            raw_v2 *= scale_factor
            raw_v3 *= scale_factor
            self.get_logger().warn(f"🚨 触发限速防呆！为防止底盘漂移，整体指令按比例压缩了 {(1.0 - scale_factor)*100:.1f}%")
        # =================================================================

        # --- B. 发送给电机 ---
        # 直接传带正负号的 int 整数！lerobot 底层会自动帮你完成 Bit 15 的转换。
        self.bus.write("Goal_Velocity", f"m{MOTOR_IDS[0]}", int(raw_v1))
        # 下发电机1目标速度
        self.bus.write("Goal_Velocity", f"m{MOTOR_IDS[1]}", int(raw_v2))
        # 下发电机2目标速度
        self.bus.write("Goal_Velocity", f"m{MOTOR_IDS[2]}", int(raw_v3))
        # 下发电机3目标速度

        # --- C. 发布真实的闭环里程计 (Odometry) ---
        
        # 1. 从总线读取 3 个电机的真实当前速度 (Present_Velocity)
        # 注意：这里需要替换为你实际使用的读取 API 函数
        try:
            read_v1 = self.bus.read("Present_Velocity", f"m{MOTOR_IDS[0]}")
            read_v2 = self.bus.read("Present_Velocity", f"m{MOTOR_IDS[1]}")
            read_v3 = self.bus.read("Present_Velocity", f"m{MOTOR_IDS[2]}")
        except Exception as e:
            self.get_logger().warn(f"读取电机速度失败: {e}")
            read_v1, read_v2, read_v3 = 0, 0, 0

        # 2. 将原始步数/秒 (ticks/s) 还原为真实的轮子线速度 (m/s)
        # 这里的 K_SPEED 必须是咱们算好的 13038！
        real_w1_m_s = read_v1 / K_SPEED
        real_w2_m_s = read_v2 / K_SPEED
        real_w3_m_s = read_v3 / K_SPEED

        # 3. 正向运动学 (Forward Kinematics)：轮子速度 -> 真实身体速度
        SQRT_3 = math.sqrt(3)
        real_vth = (real_w1_m_s + real_w2_m_s + real_w3_m_s) / (3.0 * ROBOT_RADIUS)
        real_vy = (real_w2_m_s - real_w1_m_s) / SQRT_3
        real_vx = (2.0 * real_w3_m_s - real_w1_m_s - real_w2_m_s) / 3.0

        #self.get_logger().info(f"Raw V1:{read_v1}, V2:{read_v2}, V3:{read_v3} | Calc VX:{real_vx:.3f}, VY:{real_vy:.3f}")

        # 4. 计算时间间隔 dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 5. 航位推算 (Dead Reckoning)：用真实速度进行积分！
        # 注意：这里必须使用上一步算出来的 real_vx, real_vy, real_vth
        delta_x = (real_vx * math.cos(self.th) - real_vy * math.sin(self.th)) * dt
        delta_y = (real_vx * math.sin(self.th) + real_vy * math.cos(self.th)) * dt
        delta_th = real_vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 1. 发布 TF 变换 (base_link -> odom)
        t = TransformStamped()
        # 创建 TF 变换消息
        t.header.stamp = current_time.to_msg()
        # 设置变换时间戳
        t.header.frame_id = 'odom'
        # 父坐标系为 odom
        t.child_frame_id = 'base_link'
        # 子坐标系为 base_link
        t.transform.translation.x = self.x
        # 设置平移 x
        t.transform.translation.y = self.y
        # 设置平移 y
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.th)
        # 将 yaw 转为四元数作为姿态
        self.tf_broadcaster.sendTransform(t)
        # 发送 TF 变换

        # 2. 发布 Odom 话题
        odom = Odometry()
        # 创建里程计消息
        odom.header.stamp = current_time.to_msg()
        # 设置里程计时间戳
        odom.header.frame_id = 'odom'
        # 里程计父坐标系
        odom.child_frame_id = 'base_link'
        # 里程计子坐标系
        odom.pose.pose.position.x = self.x
        # 设置位置 x
        odom.pose.pose.position.y = self.y
        # 设置位置 y
        odom.pose.pose.orientation = t.transform.rotation
        # 设置姿态四元数
        self.pub_odom.publish(odom)
        # 发布 odom 消息

    def euler_to_quaternion(self, roll, pitch, yaw):
        # 将欧拉角转换为 ROS 使用的四元数
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        # 计算四元数 x 分量
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        # 计算四元数 y 分量
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        # 计算四元数 z 分量
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        # 计算四元数 w 分量
        
        # 实例化官方的 ROS 2 Quaternion 消息
        quat = Quaternion()
        # 创建四元数消息对象
        quat.x = qx
        # 赋值 x 分量
        quat.y = qy
        # 赋值 y 分量
        quat.z = qz
        # 赋值 z 分量
        quat.w = qw
        # 赋值 w 分量
        return quat
        # 返回四元数消息

def main(args=None):
    # 节点入口函数
    rclpy.init(args=args)
    # 初始化 ROS2 客户端库
    node = LeKiwiDriver()
    # 创建驱动节点实例
    try:
        # 主循环：持续处理回调与定时器
        rclpy.spin(node)
        # 进入 ROS 事件循环
    except KeyboardInterrupt:
        # 接收到 Ctrl+C 时正常退出
        pass
        # 忽略中断异常
    finally:
        # 无论如何都执行清理，确保电机停转
        # 退出时停车
        try:
            # 尝试将所有电机速度置零并关扭矩
            for i in MOTOR_IDS: 
                # 遍历所有电机 ID
                node.bus.write("Goal_Velocity", f"m{i}", 0)
                # 速度命令清零
                node.bus.write("Torque_Enable", f"m{i}", 0)
                # 关闭扭矩
        except: pass
        # 若清理过程中异常，直接忽略以保证退出流程继续
        node.destroy_node()
        # 销毁节点实例
        rclpy.shutdown()
        # 关闭 ROS2 客户端库

if __name__ == '__main__':
    # 当脚本被直接执行时进入主函数
    main()
    # 启动程序
