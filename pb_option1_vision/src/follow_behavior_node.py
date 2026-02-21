# pb_option1_vision/follow_behavior_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
import math

class FollowBehaviorNode(Node):
    def __init__(self):
        super().__init__('follow_behavior_node')
        
        self.declare_parameter('enable_follow', False)
        self.declare_parameter('target_object', 'cup')
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('target_area_ratio', 0.3)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('center_tolerance', 50)
        
        self.enable_follow = self.get_parameter_value('enable_follow')
        self.target_object = self.get_parameter_value('target_object')
        self.linear_speed = self.get_parameter_value('linear_speed')
        self.angular_speed = self.get_parameter_value('angular_speed')
        self.target_area_ratio = self.get_parameter_value('target_area_ratio')
        self.image_width = self.get_parameter_value('image_width')
        self.image_height = self.get_parameter_value('image_height')
        self.center_tolerance = self.get_parameter_value('center_tolerance')
        
        self.object_sub = self.create_subscription(
            String,
            '/vision/detected_object',
            self.object_callback,
            10
        )
        self.position_sub = self.create_subscription(
            Point,
            '/vision/object_position',
            self.position_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.last_update_time = self.get_clock().now()
        self.timeout = 1.0  # 1 second
        
        self.current_object = None
        self.object_position = None
        
        self.get_logger().info(f'Follow node started, target object: {self.target_object}')

    def object_callback(self, msg):
        self.current_object = msg.data
        self.last_update_time = self.get_clock().now()

    def position_callback(self, msg):
        self.object_position = msg
        self.last_update_time = self.get_clock().now()

    def control_callback(self):
        if not self.enable_follow:
            return
        
        time_since = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        if time_since > self.timeout:
            stop = Twist()
            self.cmd_vel_pub.publish(stop)
            return
        
        if self.current_object != self.target_object or self.object_position is None:
            return
        
        cmd = Twist()
        
        image_center_x = self.image_width / 2.0
        error_x = self.object_position.x - image_center_x
        
        if abs(error_x) > self.center_tolerance:
            cmd.angular.z = self.angular_speed * (error_x / image_center_x)
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        
        confidence = self.object_position.z
        if confidence > 0.5:
            cmd.linear.x = self.linear_speed * 0.8
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'Follow control: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FollowBehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()