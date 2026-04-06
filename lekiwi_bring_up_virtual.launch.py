import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ==========================================
    # 🗺️ 核心路径配置
    # ==========================================
    urdf_path = '/home/kang/Documents/ros2_lekiwi/src/lekiwi_vision/urdf/lekiwi_robot.urdf.xacro' 
    rviz_path = '/home/kang/Documents/ros2_lekiwi/lekiwi.rviz'
    
    # 💥 新增：获取 YOLO 参数配置文件路径（救命稻草）
    lekiwi_vision_dir = get_package_share_directory('lekiwi_vision')
    yolo_params_file = os.path.join(lekiwi_vision_dir, 'config', 'yolo_params.yaml')

    # Xacro 引擎解析
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # ==========================================
    # 📷 唤醒 RealSense 真实相机之眼
    # ==========================================
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    start_camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'rgb_camera.color_profile': '640x480x15',
            'enable_depth': 'true',
            'depth_module.depth_profile': '640x480x15',
            'align_depth.enable': 'true',
            'initial_reset': 'true',
            # 💥 彻底封杀无关传感器，保底 USB 带宽
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false'
        }.items()
    )

    # 1. 骨架引擎
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. RViz2 (⚠️注意：如果没有连接物理显示器，请注释掉这行，否则会报 Exit Code -6)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_path]
    )

    # 3. 🦾 机械臂硬件驱动 (ttyACM1)
    arm_driver_node = Node(
        package='lekiwi_driver',
        executable='lekiwi_arm_driver',
        output='screen'
    )

    # 4. 🛞 底盘与云台硬件驱动 (ttyACM0)
    base_driver_node = Node(
        package='lekiwi_driver',
        executable='lekiwi_base_driver',
        output='screen'
    )

    # 5. 👀 YOLO 视觉节点 (💥 完美修复：注入了 yaml 参数文件)
    yolo_node = Node(
        package='lekiwi_vision',
        executable='yolo_trt_node',
        name='yolo_trt_node',
        output='screen',
        parameters=[yolo_params_file], # <--- 引擎绝对不会再迷路了！
    )

    # 6. 🧠 大脑节点
    brain_node = Node(
        package='lekiwi_driver',
        executable='lekiwi_brain_node',
        output='screen'
    )

    # 🎛️ 关节滑块面板 (⚠️注意：如果没有连接物理显示器，请注释掉这行)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    

    # 🌟 优雅的虚拟测试核心：拦截大脑指令，结合URDF补齐缺失关节（如云台），再平滑发给RViz
    virtual_joint_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='virtual_joint_publisher',
        parameters=[{
            'source_list': ['/lekiwi/joint_commands'], # 监听大脑的指令话题
            'rate': 30  # 限制频率为 30Hz，防止高速死循环撑爆网络
        }]
    )

    # 🚀 一键发射清单
    return LaunchDescription([
        rsp_node,
        start_camera_cmd, # 先拉起相机硬件
        #base_driver_node, # 拉底盘
        #arm_driver_node,  # 拉机械臂
        yolo_node,        # YOLO 挂载参数启动！
        brain_node,
        #jsp_gui_node,
        #rviz_node,
        virtual_joint_pub_node       # 虚拟测试核心
    ])