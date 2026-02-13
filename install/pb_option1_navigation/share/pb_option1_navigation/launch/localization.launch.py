import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pb_option1_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    amcl_params = LaunchConfiguration('amcl_params')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map.yaml'),
        description='Full path to map yaml')
    declare_amcl_params = DeclareLaunchArgument(
        'amcl_params',
        default_value=os.path.join(pkg_dir, 'config', 'amcl_params.yaml'),
        description='AMCL params yaml')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_log_level = DeclareLaunchArgument('log_level', default_value='info')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_amcl_params,
        declare_autostart,
        declare_log_level,
        map_server,
        amcl,
        lifecycle,
    ])
