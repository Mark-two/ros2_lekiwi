import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('lekiwi_vision')
    urdf_file = os.path.join(pkg_share, 'urdf', 'so101_arm.urdf')

    # 解析 URDF 并发布静态 TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)}]
    )

    # 默认给所有活动关节发布 0 度的假数据
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    # 启动 Foxglove
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