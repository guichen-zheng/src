#!/usr/bin/env python3
"""
仿真环境主启动文件
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    pkg_path = get_package_share_directory('my_robot_bringup')
    
    # 获取世界配置
    world_config_path = PathJoinSubstitution([
        pkg_path, 'worlds/config', 'competition_setup.yaml'
    ]).perform(context)
    
    # 读取比赛配置
    with open(world_config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    competition = LaunchConfiguration('competition').perform(context)
    comp_config = config['competitions'][competition]
    
    # Gazebo启动
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_path, 'worlds', comp_config['world_file']
            ]),
            'verbose': 'true',
            'pause': 'false',
            'use_sim_time': 'true',
        }.items()
    )
    
    nodes = [gazebo_launch]
    
    # 生成机器人节点
    for robot in comp_config.get('robots', []):
        if not robot.get('enabled', True):
            continue
            
        # 这里假设你的机器人描述包有一个spawn节点
        spawn_node = Node(
            package='my_robot_description',
            executable='spawn_robot',
            name=f'spawn_{robot["name"]}',
            output='screen',
            parameters=[{
                'robot_name': robot['name'],
                'x': str(robot['x_pose']),
                'y': str(robot['y_pose']),
                'z': str(robot['z_pose']),
                'yaw': str(robot['yaw']),
                'model': robot.get('model', 'standard_robot'),
                'color': robot.get('color', 'red'),
                'namespace': robot.get('team', 'red'),
            }]
        )
        nodes.append(spawn_node)
    
    # 导航启动
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_path, 'launch/simulation',
            'simulation_nav.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': PathJoinSubstitution([
                pkg_path, 'maps/simulation', comp_config['map_file']
            ]),
        }.items()
    )
    nodes.append(navigation_launch)
    
    return nodes

def generate_launch_description():
    # 启动参数
    competition_arg = DeclareLaunchArgument(
        'competition',
        default_value='rmul_2025',
        description='Competition type: rmul_2025, rmuc_2025, etc.'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        competition_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=launch_setup),
    ])