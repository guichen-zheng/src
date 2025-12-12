#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    navigation_core = Node(
        package='my_robot_bringup',
        executable='navigation_core',
        name='navigation_core',
        output='screen'
    )
    
    return LaunchDescription([
        navigation_core
    ])
