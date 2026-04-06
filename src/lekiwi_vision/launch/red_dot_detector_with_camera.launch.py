import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
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
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'initial_reset': 'true' # Jetson/Linux 防 USB 死锁神参
        }.items()
    )

    # ==========================================
    # 👀 红点检测节点
    # ==========================================
    red_dot_detector_node = Node(
        package='lekiwi_vision',
        executable='red_dot_detector.py',
        name='red_dot_detector',
        output='screen'
    )

    return LaunchDescription([
        start_camera_cmd,
        red_dot_detector_node
    ])
