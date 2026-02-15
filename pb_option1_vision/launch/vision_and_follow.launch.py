from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression  # 添加PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成launch描述"""
    
    # 包路径
    pkg_share = FindPackageShare('pb_option1_vision')
    
    # 配置文件路径
    detector_params = PathJoinSubstitution([
        pkg_share, 'config', 'detector_params.yaml'
    ])
    
    follow_params = PathJoinSubstitution([
        pkg_share, 'config', 'follow_params.yaml'
    ])
    
    # ============================================
    # Launch参数
    # ============================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间 (仿真环境设为true)'
    )
    
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='command',
        description='运行模式: command(物体识别控制) 或 follow(跟随模式)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz'
    )
    
    # ============================================
    # 节点配置
    # ============================================
    
    # 物体检测节点
    object_detector_node = Node(
        package='pb_option1_vision',
        executable='object_detector.py',
        name='object_detector_node',
        output='screen',
        parameters=[
            detector_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )
    
    # 命令解释节点 (物体识别控制模式)
    # 命令解释节点 (物体识别控制模式)
    command_interpreter_node = Node(
        package='pb_option1_vision',
        executable='command_interpreter_node.py',
        name='command_interpreter_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(
        PythonExpression(["'", LaunchConfiguration('mode'), "' == 'command'"])
        ),
        emulate_tty=True
)
    
    # 跟随行为节点 (跟随模式)
    # 跟随行为节点 (跟随模式)
    follow_behavior_node = Node(
        package='pb_option1_vision',
        executable='follow_behavior_node.py',
        name='follow_behavior_node',
        output='screen',
        parameters=[
            follow_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'enable_follow': True}  # 跟随模式下启用
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mode'), "' == 'follow'"])
        ),
        emulate_tty=True
    )
    
    # RViz (可选)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'vision.rviz'])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # ============================================
    # Launch描述
    # ============================================
    
    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        mode_arg,
        use_rviz_arg,
        
        # 节点启动
        object_detector_node,
        command_interpreter_node,
        follow_behavior_node,
        rviz_node,
    ])
