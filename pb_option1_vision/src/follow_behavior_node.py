#!/usr/bin/env python3
"""
物体跟随节点
功能: 控制机器人跟随检测到的物体
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist


class FollowBehaviorNode(Node):
    def __init__(self):
        super().__init__('follow_behavior_node')
        
        # 参数
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('center_tolerance', 50)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.image_width = self.get_parameter('image_width').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        
        # 当前状态
        self.current_object = None
        self.object_position = None
        self.last_update = self.get_clock().now()
        
        # 订阅
        self.object_sub = self.create_subscription(
            String, '/vision/detected_object', self.object_callback, 10)
        self.position_sub = self.create_subscription(
            Point, '/vision/object_position', self.position_callback, 10)
        
        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 控制定时器
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('跟随节点已启动')
        
    def object_callback(self, msg):
        self.current_object = msg.data
        self.last_update = self.get_clock().now()
        
    def position_callback(self, msg):
        self.object_position = msg
        self.last_update = self.get_clock().now
        
    def control_loop(self):
        # 检查超时
        if (self.get_clock().now() - self.last_update).nanoseconds / 1e9 > 1.0:
            self.stop_robot()
            return
        
        if not self.object_position:
            return
        
        # 计算控制命令
        twist = Twist()
        image_center = self.image_width / 2.0
        error_x = self.object_position.x - image_center
        
        # 角速度 - 对齐物体
        if abs(error_x) > self.center_tolerance:
            twist.angular.z = -self.angular_speed * (error_x / image_center)
        
        # 线速度 - 接近物体
        twist.linear.x = self.linear_speed * 0.5
        
        self.cmd_vel_pub.publish(twist)
        
    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FollowBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
