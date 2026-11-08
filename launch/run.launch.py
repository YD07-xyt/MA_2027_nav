#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取功能包 share 目录
    pkg_share = get_package_share_directory('ma_nav')
    
    # 默认配置文件路径（使用包内相对路径，更健壮）
    default_config = os.path.join(pkg_share, 'config', 'planner.yaml')

    # 声明参数，允许外部覆盖
    params_file_arg = DeclareLaunchArgument(
        'planner_params_path',
        default_value=default_config,
        description='Path to planner YAML config file'
    )
    
    # 全局规划节点
    ma_nav_node = Node(
        package='ma_nav',
        executable='ma_nav',
        name='ma_nav_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'planner_params_path': LaunchConfiguration('planner_params_path')
        }]
    )
    
    # 启动 RViz2 可视化
    rviz_config = os.path.join(pkg_share, 'config', 'nav.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        params_file_arg,
        ma_nav_node,
        rviz_node,  
    ])