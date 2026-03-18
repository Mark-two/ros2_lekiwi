import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('lekiwi_vision')
    urdf_file = os.path.join(pkg_share, 'urdf', 'lekiwi_robot.urdf.xacro')

    # 大管家 1：解析 xacro 模型并广播给全系统
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)}]
    )

    # 大管家 2：在没有真实硬件反馈前，先给所有活动关节发布个 0 度的假数据，防止 TF 树断裂
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # Foxglove Bridge：让 Foxglove Studio 能通过 WebSocket 连接 ROS2，
    # 并开启 asset 服务，使 Foxglove 能获取 package:// 路径下的 STL 网格文件
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'send_buffer_limit': 10000000,
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                             'services', 'connectionGraph', 'assets'],
            'asset_uri_allowlist': ['package://.*'],
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        foxglove_bridge_node,
    ])