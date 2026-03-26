#!/usr/bin/env python3
import sys
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# --- 尝试导入 LeRobot 依赖包 ---
try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ImportError:
    candidate_src = Path(__file__).resolve().parents[4] / "lerobot" / "src"
    if candidate_src.exists() and str(candidate_src) not in sys.path:
        sys.path.insert(0, str(candidate_src))
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode

# --- 硬件端口配置 (需结合实际 udev 规则调整) ---
ARM_PORT = '/dev/ttyACM1'  
ARM_IDS = [1, 2, 3, 4, 5, 6] 

class LeKiwiArmDriver(Node):
    def __init__(self):
        super().__init__('lekiwi_arm_driver')
        
        # 1. 建立 ROS 2 发射与接收链路
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.sub_joint_cmds = self.create_subscription(
            JointState, '/lekiwi/joint_commands', self.joint_cmd_callback, 10)
        
        # 2. 定义 URDF 里的关节名 (严格对应物理 ID 1~6)
        self.joint_names = [
            'shoulder_pan',       # ID 1
            'shoulder_lift',      # ID 2
            'elbow_flex',         # ID 3
            'wrist_flex',         # ID 4
            'wrist_roll',         # ID 5
            'gripper'             # ID 6
        ]
        
        # 字典映射：收到指令名字，瞬间知道操作哪个 ID
        self.joint_name_to_id = {name: ARM_IDS[i] for i, name in enumerate(self.joint_names)}
        
        # 3. 记录上一帧状态，用于“白嫖”计算速度
        self.last_time = self.get_clock().now()
        self.last_positions = [0.0] * len(ARM_IDS)
        
        # 4. 启动总线与控制循环
        self.init_motors()
        self.create_timer(0.02, self.control_loop) # 50Hz 刷屏模式

        # 🧠 零点与软限位标定字典 (白嫖自 LeRobot)
        # 格式：'URDF关节名': (零点绝对值, 方向乘数, 最小物理限位, 最大物理限位)
        self.calib_map = {
            'shoulder_pan':     (1999,  1.0,  802, 3244), # m1: shoulder_pan
            'shoulder_lift':    (1996,  1.0,  885, 3286), # m2: shoulder_lift
            'elbow_flex':       (1907,  1.0,  863, 3041), # m3: elbow_flex
            'wrist_flex':       (2013,  1.0,  804, 3145), # m4: wrist_flex
            'wrist_roll':       (2039,  1.0,  175, 4001), # m5: wrist_roll
            'gripper':          (1422,  1.0, 1527, 2999)  # m6: gripper
        }

    def init_motors(self):
        self.get_logger().info(f"正在连接机械臂总线 {ARM_PORT} ...")
        arm_config = {}
        # 创建 Motor 对象，指定型号和归一化模式（根据实际电机型号调整）
        for mid in ARM_IDS:
            arm_config[f"m{mid}"] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
            
        self.bus = FeetechMotorsBus(port=ARM_PORT, motors=arm_config)
        self.bus.connect()

        # 全部配置为位置模式 (Mode 0)
        for mid in ARM_IDS:
            name = f"m{mid}"
            self.bus.write("Lock", name, 0)
            self.bus.write("Torque_Enable", name, 0)
            self.bus.write("Operating_Mode", name, 0)
            time.sleep(0.02)
            self.bus.write("Lock", name, 1)
            self.bus.write("Torque_Enable", name, 1)
            
        self.get_logger().info("✅ 机械臂总线初始化完成！")

    def joint_cmd_callback(self, msg):
        """ 接收上层运动指令，闪电下发给物理舵机 """
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_name_to_id:
                # ... (在 if joint_name in self.joint_to_motor_map: 里面)
                target_rad = msg.position[i] 
                
                # 💥 查字典：拿到该关节的 4 个本命参数
                center_offset, direction, min_limit, max_limit = self.calib_map[joint_name]
                
                # 💥 终极运动学逆解：加上方向修正和真实的物理零点偏置
                raw_pos = int((target_rad * direction * 2048.0 / math.pi) + center_offset)
                
                # 🛡️ 工业级软限位保护：用真实测量出的极限值死死卡住！绝不撞机！
                raw_pos = max(min_limit, min(max_limit, raw_pos))
                
                physical_id = self.joint_name_to_id[joint_name]
                
                try:
                    self.bus.write("Goal_Position", f"m{physical_id}", raw_pos, normalize=False)
                except Exception as e:
                    self.get_logger().error(f"指令下发失败 {joint_name}: {e}")

    def control_loop(self):
        """ 50Hz 疯狂读取真实角度，广播给系统 """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.header.frame_id = "base_link"
        joint_msg.name = self.joint_names
        
        positions = []
        velocities = []
        
        for i, mid in enumerate(ARM_IDS):
            try:
                raw_pos = self.bus.read("Present_Position", f"m{mid}", normalize=False)
                
                # 💥 查字典
                joint_name = self.joint_names[i]
                center_offset, direction, _, _ = self.calib_map[joint_name]
                
                # 💥 终极运动学正解：减去真实零点，乘上方向，还原纯净弧度！
                rad_pos = (raw_pos - center_offset) * (math.pi / 2048.0) * direction
                
            except Exception as e:
                self.get_logger().warn(f"读取异常 ID {mid}: {e}")
                rad_pos = self.last_positions[i] # 读不到就用上一帧的数据顶住
                
            positions.append(rad_pos)
            
            # 软求导计算速度 (rad/s)
            vel = (rad_pos - self.last_positions[i]) / dt if dt > 0 else 0.0
            velocities.append(vel)
            
        self.last_positions = positions
        
        joint_msg.position = positions
        joint_msg.velocity = velocities
        
        self.pub_joint_states.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeKiwiArmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()