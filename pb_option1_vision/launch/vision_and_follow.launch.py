import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        pkg_share = get_package_share_directory('pb_option1_vision')
        rviz_config_path = os.path.join(pkg_share, 'rviz', 'vision_config.rviz')
        node1=Node(
            package='pb_option1_vision',
            executable='object_detector',
            name='object_detector',
            output='screen'
        )
        node2=Node(
            package='pb_option1_vision',
            executable='follow_behavior',
            name='follow_behavior',
            output='screen'
        )
        node3=Node(
            package='pb_option1_vision',
            executable='command_interpreter',
            name='command_interpreter',
            output='screen'
        )
        rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
        )
        return LaunchDescription([
        node1,
        node2,
        node3,
        rviz_node
        ])