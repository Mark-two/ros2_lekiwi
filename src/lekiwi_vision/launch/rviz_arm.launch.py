import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('lekiwi_vision')
    urdf_file = os.path.join(pkg_share, 'urdf', 'so101_arm.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str)}],
        # 让 robot_state_publisher 从真实臂驱动的 topic 获取关节状态
        remappings=[('/joint_states', '/lekiwi/arm_joint_states')]
    )

    # 无真实臂时的后备：GUI 滑条也发到同一 topic
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        remappings=[('/joint_states', '/lekiwi/arm_joint_states')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
