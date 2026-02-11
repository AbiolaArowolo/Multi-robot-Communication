#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TurtleTalker(Node):
    def __init__(self):
        super().__init__('turtle_talker')
        
        # Publish to Husky
        self.publisher_ = self.create_publisher(String, '/husky/incoming', 10)
        
        # Subscribe from Husky
        self.subscription = self.create_subscription(
            String, '/turtlebot3/incoming', self.listener_callback, 10)
        
        # Send message every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.get_logger().info('TurtleBot3 Talker Started')
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Husky from Turtle!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Turtle sent: {msg.data}')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Turtle received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
