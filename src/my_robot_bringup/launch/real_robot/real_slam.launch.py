
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
    
    # slam_toolbox同步模式（适合实物）
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_path, 'config/real_robot', 'slam_params.yaml']),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': ''
            }
        ]
    )
    
    return LaunchDescription([
        use_sim_time,
        SetParameter(name='use_sim_time', value=False),
        slam_toolbox,
        map_server,
    ])