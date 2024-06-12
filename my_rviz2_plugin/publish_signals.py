import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SignalPublisher(Node):

    def __init__(self):
        super().__init__('signal_publisher')
        self.publisher_ = self.create_publisher(String, '/signals', 10)
        self.timer_period = 0.01  # 秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        signal_value = format(self.counter, '016b')  # 16桁の2進数に変換
        msg = String()
        msg.data = signal_value
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter = (self.counter + 1) % (1 << 16)  # 0から65535までの値を繰り返す


def main(args=None):
    rclpy.init(args=args)

    signal_publisher = SignalPublisher()

    try:
        rclpy.spin(signal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shut down
        signal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

