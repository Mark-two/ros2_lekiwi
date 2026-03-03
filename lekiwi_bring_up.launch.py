from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 静态 TF：把相机的原点强行钉在小车正前方
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.1', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'camera_link'],
        output='screen'
    )

    # 2. 相机硬件层：只管吐彩色图和深度图，绝对不准算点云！
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'align_depth.enable': True,
            'pointcloud.enable': False,   # 🔪 彻底关闭那个会崩溃的原厂滤镜！
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30',
            'rgb_camera.power_line_frequency': 2
        }]
    )

    # 3. 最强外挂：接手彩色和深度图，高帧率输出全彩点云！
    pc_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb_node',
        namespace='camera',
        output='screen',
        remappings=[
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('rgb/image_rect_color', '/camera/color/image_raw'),
            ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_raw'),
            ('points', '/camera/depth/color/points') # 完美伪装成原厂话题名
        ]
    )

    return LaunchDescription([tf_node, camera_node, pc_node])