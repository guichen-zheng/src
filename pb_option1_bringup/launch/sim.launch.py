from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_description = get_package_share_directory('pb_option1_description')
    pkg_sim = get_package_share_directory('pb_option1_sim')
    pkg_vision = get_package_share_directory('pb_option1_vision')

    # 1. 包含 description 包里的 launch 文件（负责生成并发布 robot_description）
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_description_launch.py')  # ← 确认这是正确的文件名！
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'simulation_robot',               # 对应你的 simulation_robot.sdf.xmacro
            'use_rviz': 'false',                            # 建议关闭，避免重复开 RViz
            'use_respawn': 'true',                          # 可选，节点崩溃自动重启
            'log_level': 'info',
            # 如果需要指定其他参数（如 namespace），在这里继续添加
        }.items()
    )

    #2. 启动 Gazebo（只启动世界）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gazebo_with_objects.launch.py')
        )
    )

    #3. Spawn 小车到 Gazebo（使用 description launch 发布的 /robot_description topic）
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pb_robot',
            '-z', '0.15'          # 抬高一点，避免沉入地面
        ],
        output='screen'
    )

    # 4. 真实摄像头（你电脑上的摄像头）
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
        remappings=[('/image_raw', '/image')]
    )

    # 5. 视觉 + 跟随节点
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
        description_launch,   # 放在最前面，确保 robot_description 先准备好
        gazebo,
        spawn_robot,
        usb_cam,
        object_detector,
        follow_behavior,
    ])