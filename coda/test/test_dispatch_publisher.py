import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import time

class TestDispatchPublisher(Node):
    def __init__(self):
        super().__init__('test_dispatch_publisher')
        self.pub = self.create_publisher(Int32, '/robot1/dispatch_command', 10)
        self.get_logger().info('테스트 퍼블리셔 초기화 완료')

        # 2초 후에 테스트 명령 전송
        self.timer = self.create_timer(2.0, self.publish_test_command)

    def publish_test_command(self):
        msg = Int32()
        msg.data = 0  # 경고 ON
        self.get_logger().info('테스트: 경고 ON (0) 명령 전송 중...')
        self.pub.publish(msg)

        # 5초 후 OFF 보내고 노드 종료
        time.sleep(5)
        msg.data = 1  # 경고 OFF
        self.get_logger().info('테스트: 경고 OFF (1) 명령 전송 중...')
        self.pub.publish(msg)

        self.get_logger().info('테스트 완료 → 노드 종료')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestDispatchPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
