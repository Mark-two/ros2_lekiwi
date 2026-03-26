#!/usr/bin/env python3
"""
机械臂独立连通性测试脚本
- 不依赖 ROS2，直连电机总线
- 读取所有关节当前位置
- 对 shoulder_pan (ID 1) 做小幅往返摆动验证
- 其余关节保持原位 (只读)
"""
import sys
import time
from pathlib import Path

# 尝试导入 lerobot
try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ImportError:
    candidate_src = Path(__file__).resolve().parents[1] / "lerobot" / "src"
    if candidate_src.exists() and str(candidate_src) not in sys.path:
        sys.path.insert(0, str(candidate_src))
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors.motors_bus import Motor, MotorNormMode
    except ImportError:
        print("❌ 无法导入 lerobot，请先执行: source env_lekiwi.sh")
        sys.exit(1)

# ── 硬件配置 (与 lekiwi_arm_driver.py 保持一致) ──────────────────────────────
ARM_PORT = '/dev/ttyACM1'
ARM_IDS  = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = [
    'shoulder_pan',   # ID 1
    'shoulder_lift',  # ID 2
    'elbow_flex',     # ID 3
    'wrist_flex',     # ID 4
    'wrist_roll',     # ID 5
    'gripper',        # ID 6
]
# 格式: (中心偏置, 最小限位, 最大限位)
CALIB = {
    'shoulder_pan':  (1999, 802,  3244),
    'shoulder_lift': (1996, 885,  3286),
    'elbow_flex':    (1907, 863,  3041),
    'wrist_flex':    (2013, 804,  3145),
    'wrist_roll':    (2039, 175,  4001),
    'gripper':       (1422, 1527, 2999),
}
# 摆动测试关节 (默认 shoulder_pan) 及摆动幅度 (raw encoder units，约 10°)
TEST_JOINT = 'shoulder_pan'
SWING_DELTA = 150   # encoder units ≈ 13°


def build_bus():
    motors = {f"m{mid}": Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)
              for mid in ARM_IDS}
    return FeetechMotorsBus(port=ARM_PORT, motors=motors)


def read_all(bus):
    positions = {}
    for mid, name in zip(ARM_IDS, JOINT_NAMES):
        try:
            raw = bus.read("Present_Position", f"m{mid}", normalize=False)
            positions[name] = raw
        except Exception as e:
            positions[name] = None
            print(f"  ⚠️  读取 {name} (ID {mid}) 失败: {e}")
    return positions


def send_goal(bus, joint_name, raw_pos):
    center, lo, hi = CALIB[joint_name]
    raw_pos = max(lo, min(hi, raw_pos))
    idx = JOINT_NAMES.index(joint_name)
    mid = ARM_IDS[idx]
    bus.write("Goal_Position", f"m{mid}", raw_pos, normalize=False)
    return raw_pos


def main():
    print("=" * 55)
    print("   LeKiwi 机械臂连通性 & 运动测试")
    print("=" * 55)

    # ── 1. 连接总线 ────────────────────────────────────────────
    print(f"\n[1/4] 连接总线 {ARM_PORT} ...")
    bus = build_bus()
    try:
        bus.connect()
        print("  ✅ 总线连接成功")
    except Exception as e:
        print(f"  ❌ 连接失败: {e}")
        print("  提示: 检查 USB 线缆 / udev 规则 / 端口权限 (sudo chmod 666 /dev/ttyUSB1)")
        sys.exit(1)

    # ── 2. 初始化电机（位置模式，开扭矩）────────────────────────
    print("\n[2/4] 初始化电机 ...")
    init_ok = True
    for mid in ARM_IDS:
        mname = f"m{mid}"
        try:
            bus.write("Lock",            mname, 0)
            bus.write("Torque_Enable",   mname, 0)
            bus.write("Operating_Mode",  mname, 0)  # 位置模式
            time.sleep(0.02)
            bus.write("Lock",            mname, 1)
            bus.write("Torque_Enable",   mname, 1)
            print(f"  ✅  m{mid} ({JOINT_NAMES[mid-1]}) 初始化完成")
        except Exception as e:
            print(f"  ❌  m{mid} ({JOINT_NAMES[mid-1]}) 初始化失败: {e}")
            init_ok = False
    if not init_ok:
        print("\n⚠️  部分电机初始化失败，继续测试可能不安全，已中止。")
        bus.disconnect()
        sys.exit(1)

    # ── 3. 读取当前位置 ────────────────────────────────────────
    print("\n[3/4] 读取当前关节位置 ...")
    pos_before = read_all(bus)
    for name, raw in pos_before.items():
        if raw is not None:
            center, _, _ = CALIB[name]
            deg = (raw - center) * 180.0 / 2048.0
            print(f"  {name:20s}: raw={raw:4d}  (~{deg:+6.1f}°)")
        else:
            print(f"  {name:20s}: ❌ 读取失败")

    # ── 4. 摆动测试 ───────────────────────────────────────────
    print(f"\n[4/4] 摆动测试: {TEST_JOINT} ±{SWING_DELTA} ticks (~{SWING_DELTA*180/2048:.0f}°) ...")

    origin = pos_before.get(TEST_JOINT)
    if origin is None:
        print("  ❌ 无法读取起始位置，跳过摆动测试")
    else:
        center, lo, hi = CALIB[TEST_JOINT]
        targets = [
            origin + SWING_DELTA,
            origin - SWING_DELTA,
            origin,              # 回归原位
        ]
        labels  = ["→ 正向", "→ 反向", "→ 归位"]
        success = True

        for label, tgt in zip(labels, targets):
            actual = send_goal(bus, TEST_JOINT, tgt)
            print(f"  {label}: 目标={actual}", end="", flush=True)
            time.sleep(1.0)   # 等待转动稳定

            idx = JOINT_NAMES.index(TEST_JOINT)
            mid = ARM_IDS[idx]
            try:
                read_back = bus.read("Present_Position", f"m{mid}", normalize=False)
                deg = (read_back - center) * 180.0 / 2048.0
                err = abs(read_back - actual)
                ok  = err < 50   # 允许 ±50 tick 误差
                mark = "✅" if ok else "⚠️ "
                print(f"  实际={read_back} (~{deg:+.1f}°)  误差={err}  {mark}")
                if not ok:
                    success = False
            except Exception as e:
                print(f"  ❌ 读取失败: {e}")
                success = False

        print()
        if success:
            print("🎉 摆动测试通过！机械臂运动正常。")
        else:
            print("⚠️  摆动测试存在较大误差或读取失败，请检查电机与线缆连接。")

    # ── 清理 ──────────────────────────────────────────────────
    print("\n[清理] 卸载扭矩并断开总线 ...")
    for mid in ARM_IDS:
        try:
            bus.write("Torque_Enable", f"m{mid}", 0)
        except Exception:
            pass
    bus.disconnect()
    print("  ✅ 已断开，测试结束。")


if __name__ == "__main__":
    main()
