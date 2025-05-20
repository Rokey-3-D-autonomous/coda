import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 as i32

SERVER_PUB = "/control_scenario"

class TestSever(Node):
    def __init__(self):
        super().__init__("TestSever")
        self.get_logger().info(f'{self.__class__}')

        self.server_pub = self.create_publisher(i32, SERVER_PUB, 10)

    def pub_msg(self, i):
        msg = i32()
        msg.data = i
        self.server_pub.publish(msg)

def main():
    rclpy.init()

    ts = TestSever()

    try:    
        while rclpy.ok():
            i = int(input("정수 입력: "))
            print(f'input: {i}')
            ts.pub_msg(i)
    except KeyboardInterrupt:
        pass
    finally:
        print('tester down')
        ts.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()