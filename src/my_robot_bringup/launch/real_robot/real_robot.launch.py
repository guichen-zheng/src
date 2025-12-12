#!/usr/bin/env python3
"""
实物机器人主启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_bringup')
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    map_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            pkg_path, 'maps/real_robot', 'default_map.yaml'
        ]),
        description='Map file for real robot'
    )
    
    # C++导航核心
    navigation_core = Node(
        package='my_robot_bringup',
        executable='navigation_core',
        name='navigation_core',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config/real_robot', 'nav2_params.yaml']),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'simulation_mode': False,
                'odom_topic': '/odom',
                'laser_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'map_topic': '/map',
                'max_linear_speed': 0.3,
                'max_angular_speed': 0.8,
            }
        ]
    )
    
    # 地图服务器
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )
    
    # AMCL定位
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config/real_robot', 'nav2_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # 生命周期管理器
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time,
        map_file,
        SetParameter(name='use_sim_time', value=False),
        navigation_core,
        map_server,
        amcl,
        lifecycle_manager,
    ])