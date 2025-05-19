import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import time

class DispatchCommandTestPublisher(Node):
    def __init__(self):
        super().__init__('dispatch_test_publisher')
        self.pub = self.create_publisher(Int32, '/robot1/dispatch_command', 10)

        # 3초 후 경고 ON, 5초 후 경고 OFF
        self.timer = self.create_timer(3.0, self.send_alert_on)
        self.step = 0

    def send_alert_on(self):
        if self.step == 0:
            msg = Int32()
            msg.data = 0  # 경고 ON
            self.get_logger().info('🔊 경고 ON (0) 전송')
            self.pub.publish(msg)
            self.step += 1
            self.timer.cancel()

            # OFF 타이머는 따로 설정
            self.create_timer(5.0, self.send_alert_off)

    def send_alert_off(self):
        msg = Int32()
        msg.data = 1  # 경고 OFF
        self.get_logger().info('경고 OFF (1) 전송')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DispatchCommandTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
