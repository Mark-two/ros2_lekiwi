#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np
import threading
from pathlib import Path

# Pinocchio & Pink 运动学神级库
import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import FrameTask

# ROS 2 消息与 TF
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class LekiwiBrainNode(Node):
    def __init__(self):
        super().__init__('lekiwi_brain_node')
        
        # --- 1. ROS 2 基础通信基建 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.target_sub = self.create_subscription(PointStamped, '/lekiwi_vision/target_point', self.target_callback, 10)
        self.arm_pub = self.create_publisher(JointState, '/lekiwi/joint_commands', 10)
        
        # --- 2. 状态机全局变量 ---
        self.latest_target_pos = None  # 存放底盘坐标系下的毛球 3D 坐标
        self.is_grasping = False       # 防止重复触发抓取动作的锁
        
        # 夹爪开合角度 (需根据你 URDF 里的弧度/刻度微调)
        self.GRIPPER_OPEN = 0.5   
        self.GRIPPER_CLOSE = -0.1 
        
        # 咱们驱动认的关节名字 (必须和 URDF 以及底层驱动严格一致)
        self.JOINT_NAMES = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex', 
            'wrist_flex', 'wrist_roll', 'gripper'
        ]
        
        # --- 3. 加载 URDF 与 Pink 引擎 ---
        # ⚠️ 注意：请确保这个路径能在 Jetson 上找到你的 URDF！
        urdf_path = Path("/home/kang/Documents/ros2_lekiwi/src/assets/so101_new_calib.urdf")
        if not urdf_path.exists():
            self.get_logger().error(f"❌ 找不到 URDF 文件: {urdf_path}")
            raise FileNotFoundError("URDF is missing!")
            
        self.get_logger().info("⚙️ 正在挂载 Pinocchio 物理引擎...")
        # 这一步会解析 URDF，构建机器人模型，并准备好求解 IK 的数据结构
        self.pin_robot = pin.RobotWrapper.BuildFromURDF(str(urdf_path), package_dirs=["/home/kang/Documents/ros2_lekiwi/src/assets/"])
        self.configuration = pink.Configuration(self.pin_robot.model, self.pin_robot.data, self.pin_robot.q0)
        
        # 设定末端执行器
        self.end_effector_frame = "gripper_frame_link"
        self.position_task = FrameTask(
            self.end_effector_frame,
            position_cost=1.0,
            orientation_cost=[0.1, 0.1, 0.0], # 允许 Z 轴旋转自由度
        )
        self.position_task.lm_damping = 1e-4
        
        # 记录大脑中维持的当前姿态 (弧度)
        self.current_q_rad = self.pin_robot.q0.copy()
        
        # --- 4. 启动后台战术线程 ---
        self.get_logger().info("🚀 机甲大脑加载完毕！战术线程已启动，等待 YOLO 目标...")
        # 这个线程会持续监控 self.latest_target_pos，一旦有新目标就执行抓取流程
        self.control_thread = threading.Thread(target=self.grasp_state_machine_loop)
        # 设置为守护线程，这样当 ROS 2 关闭时它会自动退出，不会阻塞程序结束
        self.control_thread.daemon = True
        self.control_thread.start()

    def target_callback(self, msg):
        """听觉中枢：接收 YOLO 发来的相机坐标，并用 TF 转化为底盘坐标"""
        if self.is_grasping:
            return # 如果正在抓，就无视新的球
            
        try:
            # 💥 咱们的“第二刀”：用 ROS 2 官方的 TF 树，完美取代你原来手写的硬编码坐标转换！
            transform = self.tf_buffer.lookup_transform(
                'base_link',             # 目标：底座坐标系
                msg.header.frame_id,     # 来源：相机坐标系
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5)
            )
            base_point = do_transform_point(msg, transform)
            
            # 更新全局目标点
            self.latest_target_pos = np.array([base_point.point.x, base_point.point.y, base_point.point.z])
            self.get_logger().info(f"🎯 雷达锁定！底盘坐标系目标: {self.latest_target_pos}")
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ TF 坐标转换失败 (尚未打通相机和底座的物理关系?): {e}")

    # ================= 核心运动控制函数 =================
    def publish_joint_cmd(self, target_q_rad):
        """将算好的弧度打包，发给 lekiwi_arm_driver.py"""
        msg = JointState()
        msg.name = self.JOINT_NAMES
        # 将 q_rad 转成 list，确保和 JOINT_NAMES 一一对应
        msg.position = target_q_rad.tolist()
        # 💥 就是这一句！赋予时间之魂！
        msg.header.stamp = self.get_clock().now().to_msg()  
        # 👇 就是这一行！！！核弹级指令！！！必須加灵魂
        msg.header.frame_id = 'base_link'
        self.arm_pub.publish(msg)

    def solve_ik_to_position(self, target_pos, max_iterations=200, tolerance=0.001):
        """Pink IK 核心求解器"""
        target_rotation = pin.utils.rpyToMatrix(0, np.pi, 0) # 夹爪朝下
        # 构建目标位姿，注意 Pink 里 SE3 是旋转矩阵 + 平移向量的组合
        target_pose = pin.SE3(target_rotation, target_pos)
        self.position_task.set_target(target_pose)
        
        dt = 0.1
        for iteration in range(max_iterations):
            velocity = solve_ik(self.configuration, [self.position_task], dt=dt, solver="proxqp", safety_break=False)
            self.configuration.integrate_inplace(velocity, dt)
            
            current_pose = self.configuration.get_transform_frame_to_world(self.end_effector_frame)
            pos_error = np.linalg.norm(current_pose.translation - target_pos)
            
            if pos_error < tolerance:
                return self.configuration.q.copy()
        return self.configuration.q.copy()

    def move_to_joints_smoothly(self, target_q_rad, steps=50, dt=0.02):
        """生成平滑轨迹并下发"""
        start_q = self.current_q_rad.copy()
        for i in range(steps):
            interp_q_rad = start_q + (target_q_rad - start_q) * ((i+1)/steps)
            self.publish_joint_cmd(interp_q_rad)
            time.sleep(dt)
        self.current_q_rad = target_q_rad.copy()

    def set_gripper(self, gripper_val, steps=20):
        """平滑控制夹爪"""
        target_q = self.current_q_rad.copy()
        target_q[5] = gripper_val # 第6个关节是夹爪
        self.move_to_joints_smoothly(target_q, steps=steps, dt=0.02)

    # ================= 状态机战术主循环 (后台线程) =================
    def grasp_state_machine_loop(self):
        # ── 坐标说明 (base_link 坐标系，单位 m) ──────────────────────────────
        # base_link 原点 = 机械臂底座安装面
        # X 轴：正前方，Y 轴：左侧，Z 轴：正上方
        #
        # 桌面上的目标球：
        #   x ≈ 0.10~0.25  (距底座 10~25cm 前方)
        #   y ≈ ±0.10       (左右偏移)
        #   z ≈ 0.02~0.04   (球心高度 ≈ 球半径，紧贴桌面)
        #                    ⚠️  z=0.15 是在空中 15cm 处，不是桌面！
        #
        # 夹取方式：从正上方垂直下降夹取（target_rotation = rpy(0,π,0) 使夹爪朝下）
        # ─────────────────────────────────────────────────────────────────────
        INIT_POS = np.array([0.20, 0.0, 0.12])   # 归位等待点 (正前方 20cm，高 12cm)
        DROP_POS = np.array([0.12, 0.1, 0.08])  # 放球点 (左前方，紧贴桌面)
        
        while rclpy.ok():
            if self.latest_target_pos is None:
                time.sleep(0.1)
                continue
                
            self.is_grasping = True
            ball_pos = self.latest_target_pos.copy()
            self.latest_target_pos = None # 清空，防止重复抓
            
            self.get_logger().info(f"🔥 发起代号 '捕食' 动作！目标: {ball_pos}")
            
            try:
                # 1. 移至球上方
                above_pos = ball_pos.copy()
                above_pos[2] += 0.05
                self.get_logger().info(f" -> 移动至目标上方: {above_pos}")
                q_above = self.solve_ik_to_position(above_pos)
                self.move_to_joints_smoothly(q_above)
                
                # 2. 张开爪子
                self.set_gripper(self.GRIPPER_OPEN)
                time.sleep(0.5)
                
                # 3. 直捣黄龙 (下降)
                self.get_logger().info(f" -> 垂直下降至: {ball_pos}")
                q_down = self.solve_ik_to_position(ball_pos)
                self.move_to_joints_smoothly(q_down, steps=80)
                
                # 4. 死死钳住
                self.get_logger().info(" -> 闭合夹爪")
                self.set_gripper(self.GRIPPER_CLOSE, steps=50) # 慢点夹
                time.sleep(0.5)
                
                # 5. 提拔
                self.get_logger().info(f" -> 提起至: {above_pos}")
                self.move_to_joints_smoothly(q_above)
                
                # 6. 丢弃到回收站
                self.get_logger().info(f" -> 放置到指定区域: {DROP_POS}")
                q_drop = self.solve_ik_to_position(DROP_POS)
                self.move_to_joints_smoothly(q_drop)
                self.set_gripper(self.GRIPPER_OPEN)
                time.sleep(0.5)
                
                # 7. 归位
                self.get_logger().info(f" -> 归位安全区: {INIT_POS}")
                q_init = self.solve_ik_to_position(INIT_POS)
                self.move_to_joints_smoothly(q_init)
                
            except Exception as e:
                self.get_logger().error(f"❌ 抓取过程出现严重计算故障: {e}")
            
            self.get_logger().info("✅ 一轮抓取结束！重新开放雷达锁定...")
            self.is_grasping = False

def main(args=None):
    rclpy.init(args=args)
    node = LekiwiBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()