#!/usr/bin/env python3
"""
通用导航核心启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # 启动参数
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    simulation_mode = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode'
    )
    
    # 导航核心节点
    navigation_core = Node(
        package='my_robot_bringup',
        executable='navigation_core',
        name='navigation_core',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'simulation_mode': LaunchConfiguration('simulation_mode'),
        }]
    )
    
    return LaunchDescription([
        use_sim_time,
        simulation_mode,
        navigation_core,
    ])