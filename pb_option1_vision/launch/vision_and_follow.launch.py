from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pb_option1_vision',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        Node(
            package='pb_option1_vision',
            executable='follow_behavior',
            name='follow_behavior',
            output='screen'
        ),
        Node(
            package='pb_option1_vision',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen'
        ),
    ])