#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.publisher_ = self.create_publisher(String, '/signals', 10)
        self.timer = self.create_timer(0.4, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.generate_random_signal()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def generate_random_signal(self):
        return ''.join(random.choice('01') for _ in range(12))

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
