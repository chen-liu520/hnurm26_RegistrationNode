import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('registration'),
        'rviz',
        'relocalization_view.rviz'
    )
    
    return LaunchDescription([
        # 重定位节点
        Node(
            package='registration',
            executable='registration_node',
            name='relocalization_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('registration'),
                'params',
                'default.yaml'
            )]
        ),
        # RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'
        )
    ])
