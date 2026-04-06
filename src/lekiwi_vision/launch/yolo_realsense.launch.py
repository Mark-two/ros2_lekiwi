# 导入 os 模块，用于文件路径拼接等操作系统相关功能
import os
# 从 ament_index_python 包导入 get_package_share_directory，用于获取 ROS2 包的 share 目录路径
from ament_index_python.packages import get_package_share_directory
# 从 launch 模块导入 LaunchDescription 类，用于定义启动描述
from launch import LaunchDescription
# 从 launch_ros.actions 导入 Node 类，用于声明要启动的 ROS2 节点
from launch_ros.actions import Node
# 从 launch.actions 导入 IncludeLaunchDescription，用于在本 launch 文件中包含并执行另一个 launch 文件
from launch.actions import IncludeLaunchDescription
# 从 launch.launch_description_sources 导入 PythonLaunchDescriptionSource，用于指定被包含的 launch 文件是 Python 格式
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 定义 generate_launch_description 函数，ROS2 launch 系统会自动调用此函数来获取启动描述
def generate_launch_description():
    # 1. 找到咱们自己的包和 YAML 配置文件的绝对路径
    # 获取 lekiwi_vision 包的 share 目录绝对路径
    lekiwi_vision_dir = get_package_share_directory('lekiwi_vision')
    # 拼接得到 YOLO 参数配置文件 yolo_params.yaml 的完整路径
    yolo_params_file = os.path.join(lekiwi_vision_dir, 'config', 'yolo_params.yaml')

    # 2. 找到 RealSense 官方包的启动文件路径
    # 获取 realsense2_camera 官方包的 share 目录绝对路径
    realsense_dir = get_package_share_directory('realsense2_camera')
    # 拼接得到 RealSense 官方 launch 文件 rs_launch.py 的完整路径
    realsense_launch_file = os.path.join(realsense_dir, 'launch', 'rs_launch.py')

    # ==========================================
    # 动作 A：拉起 RealSense 相机底层驱动
    # ==========================================
    # 创建 IncludeLaunchDescription 动作，用于包含并启动 RealSense 相机驱动的 launch 文件
    start_camera_cmd = IncludeLaunchDescription(
        # 指定被包含的 launch 文件来源为 Python 格式的 realsense launch 文件
        PythonLaunchDescriptionSource(realsense_launch_file),
        # 传递 launch 参数给 RealSense launch 文件
        launch_arguments={
            # 把 'rgb_camera.profile' 改成 'rgb_camera.color_profile'
            'rgb_camera.color_profile': '640x480x15',
            # 关闭深度相机，找毛球场景暂时用不到深度信息，节省算力
            'enable_depth': 'true',
            'depth_module.depth_profile': '640x480x15',
            
            # --- 💥 核心：将深度图的像素严丝合缝地对齐到彩色图上 ---
            'align_depth.enable': 'true',
            # --- 🚀 新增的 Jetson 专属救命参数 ---
            # 1. 启动前强行重置一次相机硬件，清空所有死锁的 USB 寄存器
            'initial_reset': 'true',
            # 💥 修正：Humble 版本开启孔洞填充滤镜的正确语法
            #'hole_filling_filter.enable': 'true'
        }.items()
    )

    # ==========================================
    # 动作 B：拉起咱们的 C++ GPU 推理节点
    # ==========================================
    # 创建 Node 动作，用于启动 YOLO TensorRT 推理节点
    start_yolo_cmd = Node(
        # 指定节点所属的 ROS2 包名为 lekiwi_vision
        package='lekiwi_vision',
        # 指定要运行的可执行文件名为 yolo_trt_node
        executable='yolo_trt_node',
        # 设置节点在 ROS2 中的名称为 yolo_trt_node
        name='yolo_trt_node',
        # 将节点的输出（stdout/stderr）打印到终端屏幕上
        output='screen',
        # 加载 YAML 参数文件，将其中定义的参数注入到节点中
        parameters=[yolo_params_file]
    )

    # 3. 把这俩动作打包，一键发射！
    # 创建一个空的 LaunchDescription 对象，作为本 launch 文件的启动描述容器
    ld = LaunchDescription()
    # 将 RealSense 相机驱动启动动作添加到启动描述中
    ld.add_action(start_camera_cmd)
    # 将 YOLO TensorRT 推理节点启动动作添加到启动描述中
    ld.add_action(start_yolo_cmd)

    # 返回完整的启动描述，ROS2 launch 系统将按顺序执行其中的所有动作
    return ld