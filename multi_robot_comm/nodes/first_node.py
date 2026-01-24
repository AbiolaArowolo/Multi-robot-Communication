import rclpy
from rclpy.node import Node

class FirstNode(Node):
    def __init__(self):
        super().__init__('first_node')
        self.get_logger().info('First ROS2 node running')

def main(args=None):
    rclpy.init(args=args)
    node = FirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
