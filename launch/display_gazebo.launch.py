from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('test_urdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'urdf', 'guanjie2.urdf')

    # 加载 gazebo_ros 提供的默认世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # 加载机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-file', urdf_path],
        output='screen'
    )

    # 控制姿态
    pose_controller = Node(
        package='test_urdf',
        executable='pose_control.py',
        name='pose_controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
        # pose_controller
    ])
