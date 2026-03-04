import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_path = os.path.join(
        get_package_share_directory('registration'),
        'rviz',
        'relocalization_view.rviz'
    )
    
    return LaunchDescription([

        # 设置环境变量：启用彩色日志输出
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        # 设置环境变量：使用 stdout 输出（确保颜色正常显示）
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        # 设置环境变量：禁用日志缓冲，实时输出
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

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
