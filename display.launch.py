import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ==========================================================
    # 1. 定义外部参数 (让你可以用 model:=xxx 从命令行动态换模型)
    # ==========================================================
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='/home/kang/Documents/ros2_lekiwi/src/lekiwi_vision/urdf/lekiwi_robot.urdf.xacro',
        description='Xacro 模型的绝对路径'
    )

    rviz_path = '/home/kang/Documents/ros2_lekiwi/lekiwi.rviz'

    # 动态执行 Xacro 命令，把包含各种零件的 xacro 文件展平成纯文本
    robot_desc = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # ==========================================================
    # 2. 编排纯虚拟节点 (三大护法)
    # ==========================================================
    
    # 🦴 骨架引擎 (雷打不动，负责算 TF 树)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 🎛️ 关节滑块面板 (💥 核心魔法：替代真实驱动，伪造关节数据)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 👁️ RViz2 监控终端
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
    )

    # ==========================================================
    # 3. 组装并一键拉起
    # ==========================================================
    return LaunchDescription([
        model_arg,
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])