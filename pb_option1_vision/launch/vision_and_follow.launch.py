from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('pb_option1_vision')
    detector_params = PathJoinSubstitution([pkg_share, 'config', 'detector_params.yaml'])
    follow_params = PathJoinSubstitution([pkg_share, 'config', 'follow_params.yaml'])

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    mode_arg = DeclareLaunchArgument('mode', default_value='command', description='command or follow')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # 物体检测节点（C++）
    object_detector_node = Node(
        package='pb_option1_vision',
        executable='object_detector_node',  # 对应CMake中的可执行文件名
        name='object_detector_node',
        output='screen',
        parameters=[detector_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        emulate_tty=True
    )

    # 命令解释节点
    command_interpreter_node = Node(
        package='pb_option1_vision',
        executable='command_interpreter_node',
        name='command_interpreter_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'command'"])),
        emulate_tty=True
    )

    # 跟随节点
    follow_behavior_node = Node(
        package='pb_option1_vision',
        executable='follow_behavior_node',
        name='follow_behavior_node',
        output='screen',
        parameters=[follow_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'follow'"])),
        emulate_tty=True
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'vision.rviz'])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        mode_arg,
        use_rviz_arg,
        object_detector_node,
        command_interpreter_node,
        follow_behavior_node,
        rviz_node,
    ])