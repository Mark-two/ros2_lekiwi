#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import numpy as np
import threading
from pathlib import Path
from visualization_msgs.msg import Marker   # 💥 新增：导入可视化 Marker
# 💥 新增：导入底盘速度控制消息
from geometry_msgs.msg import Twist

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
        # 💥 新增：用于在 RViz2 中显示全息小球的发布器
        self.marker_pub = self.create_publisher(Marker, '/lekiwi/target_marker', 10)

        # 💥 新增：打通底盘运动神经！
        # ⚠️ 注意：请确认你的底盘驱动接收的 topic 叫什么？通常是 '/cmd_vel'
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 💥 新增：定义绝佳抓取区 (Sweet Spot)
        self.GRASP_MAX_X = 0.24  # 超过 24cm 必须往前开
        self.GRASP_MIN_X = 0.16  # 离底盘 16-24cm 是最舒服的抓取距离
        self.ALIGN_Y_TOLERANCE = 0.04 # 左右偏离超过 4cm 就要转弯微调

        # --- 2. 状态机全局变量 ---
        self.latest_target_pos = None  # 存放底盘坐标系下的毛球 3D 坐标
        self.is_grasping = False       # 防止重复触发抓取动作的锁
        
        # 夹爪开合角度 (需根据你 URDF 里的弧度/刻度微调)
        self.GRIPPER_OPEN = 1.4   
        self.GRIPPER_CLOSE = -0.1 
        
        # 咱们驱动认的关节名字 (必须和 URDF 以及底层驱动严格一致)
        self.JOINT_NAMES = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex', 
            'wrist_flex', 'wrist_roll', 'gripper'
        ]
        # 💥 新增：云台的全局状态变量（初始下倾 45 度）
        self.camera_pitch_rad = -0.785
        
        # --- 3. 加载 URDF 与 Pink 引擎 ---
        # ⚠️ 注意：请确保这个路径能在 Jetson 上找到你的 URDF！
        urdf_path = Path("/home/kang/Documents/ros2_lekiwi/src/lekiwi_vision/urdf/so101_arm.urdf")
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
            orientation_cost=[0.01, 0.01, 0.0], # 允许 Z 轴旋转自由度
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

        except Exception as e:
            self.get_logger().warn(f"⚠️ TF 坐标转换失败 (尚未打通相机和底座的物理关系?): {e}")
            return

        # 更新全局目标点
        self.latest_target_pos = np.array([base_point.point.x, base_point.point.y, base_point.point.z])

        # 将目标点投影到 RViz；若可视化失败，不影响抓取主逻辑
        try:
            self.publish_virtual_ball(base_point.point.x, base_point.point.y, base_point.point.z)
        except Exception as e:
            self.get_logger().warn(f"⚠️ 目标 Marker 发布失败: {e}")

        self.get_logger().info(f"🎯 雷达锁定！底盘坐标系目标: {self.latest_target_pos}")

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

    def solve_ik_to_position(self, target_pos, max_iterations=500, tolerance=0.005):
        """🌟 史诗级强化的 IK 求解器：带防死锁与自动重生机制"""
        # 💥 恢复姿态目标：夹爪垂直朝下
        target_rotation = pin.utils.rpyToMatrix(0, np.pi, 0) 
        target_pose = pin.SE3(target_rotation, target_pos)
        self.position_task.set_target(target_pose)
        
        # 💥 破局 1：调大 LM 阻尼！让矩阵在“奇异点”附近变软，强行挤过去
        self.position_task.lm_damping = 1e-2 

        dt = 0.1
        # 从 URDF 模型中提取关节上下限，用于钳位
        lower = self.pin_robot.model.lowerPositionLimit.copy()
        upper = self.pin_robot.model.upperPositionLimit.copy()

        # 按优先级尝试多个起始姿态：先用舒展姿态（远离关节限位），再用当前姿态
        NEUTRAL_Q = np.array([0.0, -0.5, 1.0, 0.5, 0.0, 0.0])
        start_configs = [
            ("万能起手式", NEUTRAL_Q),
            ("当前姿态", np.clip(self.current_q_rad.copy(), lower, upper)),
        ]

        for config_name, start_q in start_configs:
            test_config = pink.Configuration(self.pin_robot.model, self.pin_robot.data, start_q.copy())
            qp_failures = 0

            for iteration in range(max_iterations):
                try:
                    velocity = solve_ik(test_config, [self.position_task], dt=dt, solver="proxqp", safety_break=False)
                except Exception:
                    qp_failures += 1
                    if qp_failures > 20:
                        break  # 此起始姿态无望，换下一个
                    # 给关节加微小随机扰动，跳出死区
                    jitter = np.random.uniform(-0.05, 0.05, size=start_q.shape)
                    jittered_q = np.clip(test_config.q.copy() + jitter, lower, upper)
                    test_config = pink.Configuration(self.pin_robot.model, self.pin_robot.data, jittered_q)
                    continue

                test_config.integrate_inplace(velocity, dt)
                clamped_q = np.clip(test_config.q.copy(), lower, upper)
                test_config = pink.Configuration(self.pin_robot.model, self.pin_robot.data, clamped_q)

                current_pose = test_config.get_transform_frame_to_world(self.end_effector_frame)
                pos_error = np.linalg.norm(current_pose.translation - target_pos)

                if pos_error < tolerance:
                    self.get_logger().info(f"✨ IK [{config_name}] 求解成功！误差: {pos_error:.4f}m，迭代: {iteration}次")
                    return clamped_q.copy()

            self.get_logger().warn(f"⚠️ [{config_name}] 未收敛，尝试下一个起始姿态...")

        # 如果所有起始姿态都救不了，说明真的摸不到或者被 URDF 的 limit 卡死了
        self.get_logger().error(f"💀 IK 彻底无解！目标点 {target_pos} 超出了物理长度，或被 URDF 关节限位卡死。")
        return self.current_q_rad.copy() # 原地不动，防止发疯

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
        # 💥 战术修改：不再使用 XYZ 坐标算折叠，直接写死“折叠姿态”的关节角！
        # 这里的数组长度必须和你的机械臂关节数一致（假设是 4 或 5 个）。
        # 以下是一组典型的折叠弧度：[底座正中, 肩部后仰, 肘部极度弯曲, 手腕下垂]
        # ⚠️ 注意：你需要根据实车稍微微调这几个数字，0.0 通常是平直或居中
        FOLD_JOINTS = np.array([0.0, -1.74, 1.68, 1.3, 0.0, 0.0])
        
        DROP_POS = np.array([0.16, -0.1, -0.02])   # 放球点保持原样
        
        # 💥 核心修改：打破死锁！开局强行执行折叠归位，并疯狂散发 TF 脉冲
        self.get_logger().info("🔥 正在折叠归位并强行打通 TF 坐标树...")
        try:
            # 💥 开局先让云台就位！
            time.sleep(1)
            self.set_camera_gimbal(self.camera_pitch_rad)
            time.sleep(0.5)

            # 💥 直接把折叠关节角喂给平滑驱动函数，安全、绝对不会报错！
            self.move_to_joints_smoothly(FOLD_JOINTS, steps=50) # 步数调大点，折叠动作更柔和
            self.set_gripper(self.GRIPPER_OPEN)
            self.get_logger().info("✅ 机械臂已折叠！TF 坐标树已彻底打通！雷达开始全方位扫描...")
        except Exception as e:
            self.get_logger().error(f"❌ 初始归位动作执行失败: {e}")
        
        while rclpy.ok():
            if self.latest_target_pos is None:
                # 💥 核心修改：维持心跳！即使没有目标，也以 10Hz 发送当前关节角，保持 TF 树不崩塌
                self.publish_joint_cmd(self.current_q_rad)
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
                q_above[5] = self.GRIPPER_OPEN  # 锁定夹爪：移动过程中保持张开
                self.move_to_joints_smoothly(q_above)
                
                # 2. 张开爪子
                self.set_gripper(self.GRIPPER_OPEN)
                time.sleep(0.5)
                
                # [X, Y, Z] -> X 加 0.015 就是强行让手臂再往前探 1.5 厘米！
                # 如果你觉得还差一点点，就改成 0.02
                TCP_OFFSET = np.array([0.00, 0.0, -0.01]) 
                actual_target = ball_pos + TCP_OFFSET

                # 3. 直捣黄龙 (下降)
                self.get_logger().info(f" -> 垂直下降至: {actual_target}")
                q_down = self.solve_ik_to_position(actual_target)
                q_down[5] = self.GRIPPER_OPEN  # 锁定夹爪：下降过程中保持张开
                self.move_to_joints_smoothly(q_down, steps=80)

                # 4. 死死钳住
                self.get_logger().info(" -> 闭合夹爪")
                self.set_gripper(self.GRIPPER_CLOSE, steps=50) # 慢点夹
                time.sleep(0.5)
                
                # 5. 提拔
                self.get_logger().info(f" -> 提起至: {above_pos}")
                q_above[5] = self.GRIPPER_CLOSE  # 锁定夹爪：提起过程中保持闭合
                self.move_to_joints_smoothly(q_above)
                
                # 6. 丢弃到回收站
                self.get_logger().info(f" -> 放置到指定区域: {DROP_POS}")
                q_drop = self.solve_ik_to_position(DROP_POS)
                q_drop[5] = self.GRIPPER_CLOSE  # 锁定夹爪：搬运过程中保持闭合
                self.move_to_joints_smoothly(q_drop)
                self.set_gripper(self.GRIPPER_OPEN)
                time.sleep(0.5)
                
                # 7. 归位
                # 💥 直接把折叠关节角喂给平滑驱动函数，安全、绝对不会报错！
                self.move_to_joints_smoothly(FOLD_JOINTS, steps=50) # 步数调大点，折叠动作更柔和
                self.set_gripper(self.GRIPPER_OPEN)
                self.get_logger().info("✅ 机械臂已折叠！TF 坐标树已彻底打通！雷达开始全方位扫描...")
                
            except Exception as e:
                self.get_logger().error(f"❌ 抓取过程出现严重计算故障: {e}")
            
            self.get_logger().info("✅ 一轮抓取结束！重新开放雷达锁定...")
            self.is_grasping = False
    
    def set_camera_gimbal(self, pitch_rad):
        """
        📷 独立的云台控制接口！
        想让相机抬头低头，直接调这个函数即可。
        """
        self.camera_pitch_rad = pitch_rad
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # 只发送相机关节，不干扰手臂
        msg.name = ['camera_pitch_joint']
        msg.position = [float(self.camera_pitch_rad)]
        
        self.arm_pub.publish(msg)
        self.get_logger().info(f"📷 云台已响应，当前俯仰角: {pitch_rad:.2f} rad")

    def publish_virtual_ball(self, x, y, z):
        """将目标点具象化为 RViz2 中的 3D 红球"""
        marker = Marker()
        marker.header.frame_id = "base_link" # 锚定在底盘上
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 坐标赋值
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        
        # 设置小球大小为 4cm
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        
        # 设为半透明的亮红色
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.marker_pub.publish(marker)

    def chassis_approach(self, target_x, target_y):
        """🌟 P-Controller 底盘雷达微调系统"""
        msg = Twist()
        
        # 1. 算转向 (Y 轴偏差越大，转得越快)
        if abs(target_y) > self.ALIGN_Y_TOLERANCE:
            # 目标在左边(Y为正)，需要左转(正角速度)；在右边则右转
            msg.angular.z = max(-0.5, min(0.5, target_y * 3.0)) # 这里的 3.0 就是 Kp_w
            self.get_logger().info(f"🔄 修正航向... 转向速度: {msg.angular.z:.2f}")
        
        # 2. 算前进 (X 轴距离太远，就往前开)
        if target_x > self.GRASP_MAX_X:
            # 距离目标 20cm (0.2) 算是完美距离
            dist_error = target_x - 0.20
            msg.linear.x = max(0.0, min(0.1, dist_error * 0.5)) # 这里的 0.5 就是 Kp_v，限速 0.1m/s
            self.get_logger().info(f"🚙 逼近目标... 前进速度: {msg.linear.x:.2f}")

        # 下发指令，让车动一小步
        self.cmd_vel_pub.publish(msg)
        
        # 💥 极其关键：走一小步，等 0.5 秒，然后立刻刹车！
        # 为什么要刹车？因为车在动的时候摄像头会晃，YOLO 坐标会飞！必须停下来看！
        time.sleep(0.5)
        self.cmd_vel_pub.publish(Twist()) # 发送全 0 刹车指令

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