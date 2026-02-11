#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HuskyTalker(Node):
    def __init__(self):
        super().__init__('husky_talker')
        
        # Publish to TurtleBot3
        self.publisher_ = self.create_publisher(String, '/turtlebot3/incoming', 10)
        
        # Subscribe from TurtleBot3
        self.subscription = self.create_subscription(
            String, '/husky/incoming', self.listener_callback, 10)
        
        # Send message every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('Husky Talker Started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Turtle from Husky!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Husky sent: {msg.data}')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Husky received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HuskyTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
