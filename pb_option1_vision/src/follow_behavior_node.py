#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist


class FollowBehaviorNode(Node):
    def __init__(self):
        super().__init__('follow_behavior_node')
        
        # 声明参数
        self.declare_parameter('enable_follow', False)  # 默认关闭,由命令解释节点控制
        self.declare_parameter('target_object', 'cup')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('target_area_ratio', 0.3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('center_tolerance', 50)
        
        # 获取参数
        self.enable_follow = self.get_parameter('enable_follow').value
        self.target_object = self.get_parameter('target_object').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.target_area_ratio = self.get_parameter('target_area_ratio').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.center_tolerance = self.get_parameter('center_tolerance').value
        
        # 当前检测到的物体和位置
        self.current_object = None
        self.object_position = None
        
        # 订阅物体类型
        self.object_sub = self.create_subscription(
            String,
            '/vision/detected_object',
            self.object_callback,
            10
        )
        
        # 订阅物体位置
        self.position_sub = self.create_subscription(
            Point,
            '/vision/object_position',
            self.position_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # 创建跟随控制定时器
        self.control_timer = self.create_timer(0.1, self.follow_control)
        
        # 超时检测
        self.last_update_time = self.get_clock().now()
        self.timeout = 1.0  # 1秒无更新则停止
        
        self.get_logger().info('='*60)
        self.get_logger().info('跟随行为节点已启动')
        self.get_logger().info('='*60)
        self.get_logger().info(f'跟随模式: {"启用" if self.enable_follow else "禁用"}')
        self.get_logger().info(f'目标物体: {self.target_object}')
        self.get_logger().info(f'线速度: {self.linear_speed} m/s')
        self.get_logger().info(f'角速度: {self.angular_speed} rad/s')
        self.get_logger().info('='*60)
        
    def object_callback(self, msg):
        """接收检测到的物体类型"""
        self.current_object = msg.data
        self.last_update_time = self.get_clock().now()
        
    def position_callback(self, msg):
        """接收物体位置"""
        self.object_position = msg
        self.last_update_time = self.get_clock().now()
        
    def follow_control(self):
        """跟随控制逻辑"""
        if not self.enable_follow:
            return
        
        # 检查超时
        time_since_update = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        if time_since_update > self.timeout:
            # 超时,停止
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # 检查是否是目标物体
        if self.current_object != self.target_object:
            return
        
        # 检查是否有位置信息
        if self.object_position is None:
            return
        
        # 计算控制命令
        twist = Twist()
        
        # 图像中心
        image_center_x = self.image_width / 2.0
        
        # 物体相对于中心的偏移
        error_x = self.object_position.x - image_center_x
        
        # 角速度控制(对齐)
        if abs(error_x) > self.center_tolerance:
            # 物体在左边 -> 左转(正角速度)
            # 物体在右边 -> 右转(负角速度)
            twist.angular.z = -self.angular_speed * (error_x / image_center_x)
            twist.angular.z = max(-self.angular_speed, min(self.angular_speed, twist.angular.z))
        
        # 线速度控制(距离)
        # 这里简化处理,始终保持一定速度前进
        # 可以根据物体大小(area_ratio)来调整距离
        confidence = self.object_position.z  # z存储的是置信度
        if confidence > 0.5:
            twist.linear.x = self.linear_speed * 0.8
        
        # 发布命令
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f'跟随控制: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FollowBehaviorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
