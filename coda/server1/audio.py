import rclpy
import time

from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Header
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

class AudioPublisher(Node):

    def __init__(self):
        super().__init__('cmd_audio_publisher')

        # create Pub
        self.publisher_ = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)

        # create timer
        # self.timer_period = 1.0
        # self.timer = self.create_timer(self.timer_period, self.publish_audio)

        self.publish_audio()

    def publish_audio(self):
        # create Msg
        msg = AudioNoteVector()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""

        # msg.notes = [
        #     AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
        #     AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        #     AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
        #     AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        # ]
        msg.notes = [
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # E5
            AudioNote(frequency=622, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # D#5
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # E5
            AudioNote(frequency=622, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # D#5
            AudioNote(frequency=659, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # E5
            AudioNote(frequency=494, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # B4
            AudioNote(frequency=587, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # D5
            AudioNote(frequency=523, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # C5
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=500_000_000)),  # A4 (조금 길게)
        ]
        msg.append = False

        self.publisher_.publish(msg)
        self.get_logger().info('삐뽀삐뽀 오디오 메시지 전송')

def main():
    print('Hi from turtlebot4_beep.')
    
    rclpy.init(args=None)
    audio_publisher = AudioPublisher()
    
    try:
        while True:
            input("엔터를 누르면 삐뽀삐뽀 메시지를 보냅니다 (Ctrl+C로 종료): ")
            audio_publisher.publish_audio()
            time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        audio_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
