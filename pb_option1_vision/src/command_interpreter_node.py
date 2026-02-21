# pb_option1_vision/command_interpreter_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandInterpreterNode(Node):
    def __init__(self):
        super().__init__('command_interpreter_node')
        
        self.object_action_map = {
            'cup': {'action': 'forward', 'linear_x': 0.3, 'angular_z': 0.0},
            'apple': {'action': 'right', 'linear_x': 0.0, 'angular_z': -0.5},
            'banana': {'action': 'left', 'linear_x': 0.0, 'angular_z': 0.5}
        }
        
        self.sub = self.create_subscription(
            String,
            '/vision/detected_object',
            self.object_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.action_pub = self.create_publisher(String, '/vision/current_action', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.last_detection_time = self.get_clock().now()
        self.stop_timeout = 2.0  # 2 seconds
        
        self.current_action = {'action': '', 'linear_x': 0.0, 'angular_z': 0.0}
        
        self.get_logger().info('Command interpreter node started')
        for obj, info in self.object_action_map.items():
            self.get_logger().info(f'  {obj} -> {info["action"]} (linear={info["linear_x"]:.1f}, angular={info["angular_z"]:.1f})')

    def object_callback(self, msg):
        if msg.data in self.object_action_map:
            self.last_detection_time = self.get_clock().now()
            self.current_action = self.object_action_map[msg.data]
            
            self.get_logger().info(f'Detected {msg.data} -> executing {self.current_action["action"]}')
            
            action_msg = String()
            action_msg.data = f'{msg.data}:{self.current_action["action"]}'
            self.action_pub.publish(action_msg)

    def timer_callback(self):
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_detection > self.stop_timeout:
            if self.current_action['linear_x'] != 0.0 or self.current_action['angular_z'] != 0.0:
                self.get_logger().info('No object detected, stopping motion')
                self.current_action = {'action': '', 'linear_x': 0.0, 'angular_z': 0.0}
            
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        if self.current_action['linear_x'] != 0.0 or self.current_action['angular_z'] != 0.0:
            twist = Twist()
            twist.linear.x = self.current_action['linear_x']
            twist.angular.z = self.current_action['angular_z']
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CommandInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()