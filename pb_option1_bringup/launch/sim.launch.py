from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_description = get_package_share_directory('pb_option1_description')
    pkg_sim = get_package_share_directory('pb_option1_sim')
    pkg_vision = get_package_share_directory('pb_option1_vision')

    # 1. 调用 description 包的 launch（处理 xacro + robot_state_publisher）
    description_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            pkg_description, 'launch', 'robot_description_launch.py'
        ])
    )

    # 2. 只启动一个 Gazebo（带物体的世界）
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            pkg_sim, 'launch', 'gazebo_with_objects.launch.py'
        ])
    )

    # 3. Spawn 小车（必须保留！因为 description 里没有）
    xacro_file = os.path.join(pkg_description, 'resource', 'xmacro', 'simulation_robot.sdf.xacro')

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', xacro_file,
            '-entity', 'pb_robot',
            '-x', '0',      # ← 世界中心 X
            '-y', '0',      # ← 世界中心 Y
            '-z', '0.25',   # ← 稍微抬高一点，防止卡地
            '-Y', '0',      # ← 朝向（弧度），0=朝正前方
            '--allow_renaming', 'true'   # 防止重名
        ],
        output='screen'
    )
    # 4. 真实摄像头 + 视觉节点
    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
        }],
        remappings=[('/image_raw', '/image')],
        output='screen'
    )

    object_detector = Node(
        package='pb_option1_vision',
        executable='object_detector',
        output='screen'
    )

    follow_behavior = Node(
        package='pb_option1_vision',
        executable='follow_behavior',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        description_launch,
        gazebo,
        spawn_robot,          # ← 必须保留
        usb_cam,
        object_detector,
        follow_behavior,
    ])