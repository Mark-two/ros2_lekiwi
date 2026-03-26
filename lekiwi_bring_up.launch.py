import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ==========================================================
    # 1. 定义绝对路径 (完全契合亢哥的真实物理路径)
    # ==========================================================
    urdf_path = '/home/kang/Documents/ros2_lekiwi/src/lekiwi_vision/urdf/so101_arm.urdf'
    rviz_path = '/home/kang/Documents/ros2_lekiwi/lekiwi.rviz'

    # 💥 极客级优化：直接在内存里把 URDF 读成纯文本字符串。
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # ==========================================================
    # 2. 编排系统节点 (五虎上将齐聚)
    # ==========================================================
    
    # 🦴 骨架引擎
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 👁️ RViz2 监控终端
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
    )

    # 🦾 真实机械臂底层驱动
    driver_node = Node(
        package='lekiwi_driver',
        executable='lekiwi_arm_driver',
        output='screen'
    )

    # 🧠 Pink 逆解大脑
    brain_node = Node(
        package='lekiwi_driver',
        executable='lekiwi_brain_node',
        output='screen'
    )

    # 👀 YOLO TensorRT 视觉推理节点 (亢哥点名的王牌)
    yolo_node = Node(
        package='lekiwi_vision',
        executable='yolo_trt_node',
        output='screen'
    )

    # ==========================================================
    # 3. 组装并交由 ROS 2 引擎一键拉起
    # ==========================================================
    return LaunchDescription([
        rsp_node,
        driver_node,  # 先拉底层物理驱动
        yolo_node,    # 拉起视觉雷达！开始疯狂扫描目标
        brain_node,   # 拉起大脑！大脑现在可以直接接收 YOLO 的目标了
        rviz_node     # 最后拉 RViz2 渲染画面
    ])