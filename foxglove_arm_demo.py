#!/usr/bin/env python3
"""
SO-101 机械臂 Foxglove 可视化 —— 逐关节诊断 + 连续演示
启动方式:
  终端1: ros2 launch lekiwi_vision display_robot.launch.py
  终端2: python3 foxglove_arm_demo.py          # 逐关节诊断模式
         python3 foxglove_arm_demo.py --wave    # 全关节摆动模式
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ── URDF 中所有活动关节 ──────────────────────────
JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
    "camera_pitch_joint",
]

JOINT_LIMITS = {
    "shoulder_pan":       (-1.91986,  1.91986),
    "shoulder_lift":      (-1.74533,  1.74533),
    "elbow_flex":         (-1.69,     1.69),
    "wrist_flex":         (-1.65806,  1.65806),
    "wrist_roll":         (-2.74385,  2.84121),
    "gripper":            (-0.174533, 1.74533),
    "camera_pitch_joint": (-0.785,    0.785),
}

# 每个关节正方向的物理预期 (用于诊断打印)
JOINT_EXPECT = {
    "shoulder_pan":  "底座水平旋转 (从俯视图看应逆时针)",
    "shoulder_lift": "肩部抬起 (上臂应向上/向后摆)",
    "elbow_flex":    "肘部弯曲 (小臂应向上/向后折)",
    "wrist_flex":    "腕部弯曲 (手腕应向上/向后翘)",
    "wrist_roll":    "手腕滚转 (夹爪绕前臂轴旋转)",
    "gripper":       "夹爪张开 (两爪分开)",
    "camera_pitch_joint": "相机俯仰 (向上抬)",
}


class ArmDemoNode(Node):
    def __init__(self, mode="diagnose"):
        super().__init__("arm_demo_node")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.mode = mode
        self.t0 = time.time()
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

        if mode == "wave":
            self.get_logger().info("🦾 全关节摆动模式启动！")
        else:
            self.get_logger().info("🔍 逐关节诊断模式启动！每个关节运动 4 秒...")
            self.diag_index = 0
            self.diag_start = time.time()
            self._print_current_joint()

    def _print_current_joint(self):
        name = JOINT_NAMES[self.diag_index]
        lo, hi = JOINT_LIMITS[name]
        expect = JOINT_EXPECT.get(name, "")
        self.get_logger().info(
            f"▶ [{self.diag_index+1}/{len(JOINT_NAMES)}] {name}  "
            f"范围[{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]  "
            f"预期: {expect}"
        )

    def tick(self):
        if self.mode == "wave":
            self._publish_wave()
        else:
            self._publish_diagnose()

    # ── 模式1: 逐关节诊断 ──
    def _publish_diagnose(self):
        t = time.time()
        elapsed = t - self.diag_start

        # 每个关节测试 4 秒，然后换下一个
        if elapsed > 4.0:
            self.diag_index += 1
            if self.diag_index >= len(JOINT_NAMES):
                self.get_logger().info("✅ 全部关节诊断完毕！重新开始...")
                self.diag_index = 0
            self.diag_start = t
            elapsed = 0.0
            self._print_current_joint()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        active_name = JOINT_NAMES[self.diag_index]
        lo, hi = JOINT_LIMITS[active_name]
        mid = (lo + hi) / 2.0
        half = (hi - lo) / 2.0

        for name in JOINT_NAMES:
            msg.name.append(name)
            if name == active_name:
                # 在限位 30% 范围内来回摆
                val = mid + half * 0.5 * math.sin(2 * math.pi * 0.5 * elapsed)
                msg.position.append(max(lo, min(hi, val)))
            else:
                # 其余关节保持零位
                msg.position.append(0.0)

        self.pub.publish(msg)

    # ── 模式2: 全关节同时摆动 ──
    def _publish_wave(self):
        t = time.time() - self.t0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        freqs  = [0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.1]
        phases = [0.0,  0.5, 1.0,  1.5, 2.0,  2.5, 0.0]
        amps   = [0.6,  0.8, 0.9,  0.7, 0.5,  0.6, 0.3]

        for i, name in enumerate(JOINT_NAMES):
            lo, hi = JOINT_LIMITS[name]
            mid = (lo + hi) / 2.0
            half = (hi - lo) / 2.0
            val = mid + half * amps[i] * math.sin(2 * math.pi * freqs[i] * t + phases[i])
            msg.name.append(name)
            msg.position.append(max(lo, min(hi, val)))

        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--wave", action="store_true", help="全关节同时摆动模式")
    args = parser.parse_args()

    rclpy.init()
    node = ArmDemoNode(mode="wave" if args.wave else "diagnose")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
