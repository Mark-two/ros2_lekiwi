#!/usr/bin/env python3
import sys
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

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

# --- 底盘物理参数配置 ---
WHEEL_RADIUS = 0.05
ROBOT_RADIUS = 0.15
K_SPEED = 13038

# --- 硬件端口配置 (需结合实际 udev 规则调整) ---
BASE_PORT = '/dev/ttyUSB0' 
BASE_WHEEL_IDS = [1, 2, 3] # 轮子 ID
BASE_CAM_ID = 4            # 相机 ID

class LeKiwiBaseDriver(Node):
    def __init__(self):
        super().__init__('lekiwi_base_driver')
        
        # 1. ROS 通讯接口
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self.control_loop) # 50Hz
        
        # 2. 状态变量
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.target_vx, self.target_vy, self.target_vth = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.wheel_angles = [0.0, 0.0, 0.0] # 记录轮子累计旋转弧度
        
        # 3. 初始化底盘总线
        self.init_motors()

    def init_motors(self):
        self.get_logger().info(f"正在连接底盘总线 {BASE_PORT} ...")
        base_config = {}
        for mid in BASE_WHEEL_IDS + [BASE_CAM_ID]:
            base_config[f"m{mid}"] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
        
        self.bus = FeetechMotorsBus(port=BASE_PORT, motors=base_config)
        self.bus.connect()
        
        # 初始化轮子 (Mode 1: 连续旋转)
        for mid in BASE_WHEEL_IDS:
            name = f"m{mid}"
            self.bus.write("Lock", name, 0)
            self.bus.write("Torque_Enable", name, 0)
            self.bus.write("Operating_Mode", name, 1) 
            time.sleep(0.02)
            self.bus.write("Acceleration", name, 50)
            self.bus.write("Lock", name, 1)
            self.bus.write("Torque_Enable", name, 1)

        # 初始化相机 (Mode 0: 位置模式)
        cam_name = f"m{BASE_CAM_ID}"
        self.bus.write("Lock", cam_name, 0)
        self.bus.write("Torque_Enable", cam_name, 0)
        self.bus.write("Operating_Mode", cam_name, 0)
        time.sleep(0.02)
        self.bus.write("Lock", cam_name, 1)
        self.bus.write("Torque_Enable", cam_name, 1)
        self.get_logger().info("✅ 底盘总线初始化完成！")

    def cmd_vel_callback(self, msg):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_vth = msg.angular.z

    def control_loop(self):
        vx, vy, vth = self.target_vx, self.target_vy, self.target_vth
        
        # --- A. 逆运动学 (底盘速度 -> 轮速指令) ---
        w1_speed_m_s = -0.5 * vx - (math.sqrt(3)/2) * vy + ROBOT_RADIUS * vth
        w2_speed_m_s = -0.5 * vx + (math.sqrt(3)/2) * vy + ROBOT_RADIUS * vth
        w3_speed_m_s =  vx + ROBOT_RADIUS * vth

        raw_v1 = w1_speed_m_s * K_SPEED
        raw_v2 = w2_speed_m_s * K_SPEED
        raw_v3 = w3_speed_m_s * K_SPEED

        # 防抖限速
        MAX_SPEED_TICKS = 2400.0
        max_target = max(abs(raw_v1), abs(raw_v2), abs(raw_v3))
        if max_target > MAX_SPEED_TICKS:
            scale = MAX_SPEED_TICKS / max_target
            raw_v1, raw_v2, raw_v3 = raw_v1 * scale, raw_v2 * scale, raw_v3 * scale

        # 下发轮速
        self.bus.write("Goal_Velocity", f"m{BASE_WHEEL_IDS[0]}", int(raw_v1))
        self.bus.write("Goal_Velocity", f"m{BASE_WHEEL_IDS[1]}", int(raw_v2))
        self.bus.write("Goal_Velocity", f"m{BASE_WHEEL_IDS[2]}", int(raw_v3))

        # --- B. 正向运动学 (读取真实速度 -> 里程计) ---
        try:
            read_v1 = self.bus.read("Present_Velocity", f"m{BASE_WHEEL_IDS[0]}")
            read_v2 = self.bus.read("Present_Velocity", f"m{BASE_WHEEL_IDS[1]}")
            read_v3 = self.bus.read("Present_Velocity", f"m{BASE_WHEEL_IDS[2]}")
        except:
            read_v1, read_v2, read_v3 = 0, 0, 0

        real_w1_m_s = read_v1 / K_SPEED
        real_w2_m_s = read_v2 / K_SPEED
        real_w3_m_s = read_v3 / K_SPEED

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        SQRT_3 = math.sqrt(3)
        real_vth = (real_w1_m_s + real_w2_m_s + real_w3_m_s) / (3.0 * ROBOT_RADIUS)
        real_vy = (real_w2_m_s - real_w1_m_s) / SQRT_3
        real_vx = (2.0 * real_w3_m_s - real_w1_m_s - real_w2_m_s) / 3.0

        self.x += (real_vx * math.cos(self.th) - real_vy * math.sin(self.th)) * dt
        self.y += (real_vx * math.sin(self.th) + real_vy * math.cos(self.th)) * dt
        self.th += real_vth * dt

        # 积分计算轮子自转角度
        self.wheel_angles[0] += (real_w1_m_s / WHEEL_RADIUS) * dt
        self.wheel_angles[1] += (real_w2_m_s / WHEEL_RADIUS) * dt
        self.wheel_angles[2] += (real_w3_m_s / WHEEL_RADIUS) * dt

        # --- C. 发布 TF 和 Odom ---
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.euler_to_quaternion(0, 0, self.th)
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        self.pub_odom.publish(odom)

        # --- D. 发布底盘与相机的 JointState ---
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [
            'wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint', 'camera_pitch_joint'
        ]
        
        # 读取相机真实角度
        try:
            cam_raw = self.bus.read("Present_Position", f"m{BASE_CAM_ID}")
            cam_rad = (cam_raw - 2048.0) * (math.pi / 2048.0)
        except:
            cam_rad = 0.0

        joint_msg.position = [
            self.wheel_angles[0], self.wheel_angles[1], self.wheel_angles[2], cam_rad
        ]
        self.pub_joint_states.publish(joint_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = qx, qy, qz, qw
        return quat

def main(args=None):
    rclpy.init(args=args)
    node = LeKiwiBaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for i in BASE_WHEEL_IDS:
                node.bus.write("Goal_Velocity", f"m{i}", 0)
                node.bus.write("Torque_Enable", f"m{i}", 0)
        except: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()