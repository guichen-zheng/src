from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_description = get_package_share_directory('pb_option1_description')
    pkg_bringup = get_package_share_directory('pb_option1_bringup')
    pkg_sim = get_package_share_directory('pb_option1_sim')
    pkg_vision = get_package_share_directory('pb_option1_vision')

    # 主模型文件（XMacro）
    xacro_file = os.path.join(pkg_description, 'resource', 'xmacro', 'simulation_robot.sdf.xmacro')

    # Gazebo（使用你已有的带物体世界）
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_sim, 'launch', 'gazebo_with_objects.launch.py']),
    )

    # Spawn 机器人
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', xacro_file, '-entity', 'pb_robot'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(xacro_file).read()}],
        output='screen'
    )

    # 真实 USB 摄像头（关键）
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video0',      # 用 ls /dev/video* 确认
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
        }],
        remappings=[('/image_raw', '/image')]
    )

    # 视觉节点
    object_detector = Node(
        package='pb_option1_vision',
        executable='object_detector',
        output='screen'
    )

    # 跟随行为节点
    follow_behavior = Node(
        package='pb_option1_vision',
        executable='follow_behavior_node',   # 如果名字不同请修改
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        spawn_robot,
        robot_state_publisher,
        usb_cam,
        object_detector,
        follow_behavior,
    ])