from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_pb_option1_vision = get_package_share_directory('pb_option1_vision')

    # 物体检测节点
    object_detector = Node(
        package='pb_option1_vision',
        executable='object_detector',
        name='object_detector',
        parameters=[os.path.join(pkg_pb_option1_vision, 'config', 'detector_params.yaml')],
        remappings=[('/image', '/demo/image')]  # remap到真实摄像头topic
    )

    # 跟随行为节点
    follow_behavior = Node(
        package='pb_option1_vision',
        executable='follow_behavior',
        name='follow_behavior',
        parameters=[os.path.join(pkg_pb_option1_vision, 'config', 'follow_params.yaml')]
    )

    # 命令解释节点
    command_interpreter = Node(
        package='pb_option1_vision',
        executable='command_interpreter',
        name='command_interpreter'
    )

    # RViz（README中提到已设置）
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_pb_option1_vision, 'rviz', 'vision.rviz')]  # 假设有rviz config
    )

    return LaunchDescription([
        object_detector,
        follow_behavior,
        command_interpreter,
        rviz
    ])