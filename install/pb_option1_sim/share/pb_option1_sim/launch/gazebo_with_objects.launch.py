from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_pb_option1_description = get_package_share_directory('pb_option1_description')
    pkg_pb_option1_sim = get_package_share_directory('pb_option1_sim')

    # Gazebo启动
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_pb_option1_sim, 'worlds', 'simple_env.world')}.items()
    )

    # 生成机器人描述（从pb_option1_description）
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pb_option1_description, 'launch', 'robot_description_launch.py')
        )
    )

    # spawn机器人到Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'pb_option1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_description_launch,
        spawn_entity
    ])