# launch/find_lidar_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    pkg_share = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[0], 'share', 'cpp_get_pkg')
    rviz_config = os.path.join(pkg_share, 'rviz', 'find_lidar.rviz')

    return LaunchDescription([
        Node(
            package='cpp_get_pkg',
            executable='FindLidar',  # 与 CMakeLists.txt 中 add_executable 一致
            name='find_lidar',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
