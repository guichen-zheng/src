#!/usr/bin/env python3
"""
仿真环境导航启动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_bringup')
    
    # 启动参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    map_file = DeclareLaunchArgument(
        'map',
        description='Map file path'
    )
    
    # C++导航核心节点
    navigation_core = Node(
        package='my_robot_bringup',
        executable='navigation_core',
        name='navigation_core',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config/simulation', 'nav2_params.yaml']),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'simulation_mode': True,
                'odom_topic': '/odom',
                'laser_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'map_topic': '/map',
                'goal_topic': '/goal_pose',
                'max_linear_speed': 0.5,
                'max_angular_speed': 1.0,
            }
        ]
    )
    
    # Nav2启动
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                pkg_path, 'config/simulation', 'nav2_params.yaml'
            ]),
            'autostart': 'true',
            'map': LaunchConfiguration('map'),
        }.items()
    )
    
    # SLAM启动
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_path, 'launch/simulation',
            'simulation_slam.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time,
        map_file,
        SetParameter(name='use_sim_time', value=True),
        navigation_core,
        nav2_launch,
        slam_launch,
    ])