
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_bringup')
    
    # 启动参数
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
        description='Map file path'
    )
    
    # C++导航节点 - 实物模式
    navigation_core = Node(
        package='my_robot_bringup',
        executable='navigation_core',
        name='navigation_core',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config/real_robot', 'nav2_params.yaml']),
            {
                'simulation_mode': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_topic': '/real_odom',
                'laser_topic': '/real_scan',
                'cmd_vel_topic': '/real_cmd_vel',
                'map_topic': '/map',
                'goal_topic': '/goal_pose',
                'max_linear_speed': 0.3,
                'max_angular_speed': 0.8,
                'goal_tolerance': 0.1,
                'safety_distance': 0.4,
                'emergency_stop_distance': 0.2,
            }
        ],
        # 实物机器人可能需要sudo权限
        # prefix='sudo -E',
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
    
    # 速度平滑器（实物机器人需要）
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_velocity': 0.3,
                'max_acceleration': 0.5,
                'frequency': 20.0,
            }
        ],
        remappings=[
            ('cmd_vel', '/nav_cmd_vel'),
            ('cmd_vel_smoothed', '/real_cmd_vel'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time,
        map_file,
        SetParameter(name='use_sim_time', value=False),
        navigation_core,
        map_server,
        amcl,
        velocity_smoother,
    ])