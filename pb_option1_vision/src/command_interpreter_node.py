#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CommandInterpreterNode(Node):
    
    def __init__(self):
        super().__init__('command_interpreter_node')
        
        # 物体到动作的映射
        self.object_action_map = {
            'cup': {
                'action': 'forward',
                'description': '前行',
                'linear_x': 0.3,
                'angular_z': 0.0
            },
            'apple': {
                'action': 'left',
                'description': '左转',
                'linear_x': 0.0,
                'angular_z': 0.5
            },
            'banana': {
                'action': 'right',
                'description': '右转',
                'linear_x': 0.0,
                'angular_z': -0.5
            }
        }
        
        # 当前执行的动作
        self.current_action = None
        self.current_object = None
        
        # 订阅检测到的物体
        self.object_sub = self.create_subscription(
            String,
            '/vision/detected_object',
            self.object_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 发布动作命令(用于显示)
        self.action_pub = self.create_publisher(
            String,
            '/vision/current_action',
            10
        )
        
        # 创建定时器用于执行动作
        self.action_timer = self.create_timer(0.1, self.execute_action)
        
        # 无检测超时停止
        self.last_detection_time = self.get_clock().now()
        self.stop_timeout = 2.0  # 2秒无检测则停止
        
        self.get_logger().info('='*60)
        self.get_logger().info('命令解释节点已启动')
        self.get_logger().info('='*60)
        self.get_logger().info('物体 -> 动作映射:')
        for obj, action_info in self.object_action_map.items():
            self.get_logger().info(
                f'  {obj.upper():8s} -> {action_info["description"]} '
                f'(linear: {action_info["linear_x"]}, '
                f'angular: {action_info["angular_z"]})'
            )
        self.get_logger().info('='*60)
        
    def object_callback(self, msg):
        """接收检测到的物体"""
        detected_object = msg.data
        
        if detected_object in self.object_action_map:
            # 更新检测时间
            self.last_detection_time = self.get_clock().now()
            
            # 如果检测到新的物体,更新动作
            if detected_object != self.current_object:
                self.current_object = detected_object
                action_info = self.object_action_map[detected_object]
                self.current_action = action_info
                
                self.get_logger().info(
                    f'检测到 {detected_object.upper()} -> '
                    f'执行动作: {action_info["description"]}'
                )
                
                # 发布动作消息
                action_msg = String()
                action_msg.data = f"{detected_object}:{action_info['action']}"
                self.action_pub.publish(action_msg)
    
    def execute_action(self):
        """执行当前动作"""
        # 检查是否超时
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_detection > self.stop_timeout:
            # 超时,停止机器人
            if self.current_action is not None:
                self.get_logger().info(' 未检测到物体,停止运动', throttle_duration_sec=2.0)
                self.current_action = None
                self.current_object = None
            
            # 发送停止命令
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # 如果有当前动作,执行它
        if self.current_action:
            twist = Twist()
            twist.linear.x = self.current_action['linear_x']
            twist.angular.z = self.current_action['angular_z']
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CommandInterpreterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
