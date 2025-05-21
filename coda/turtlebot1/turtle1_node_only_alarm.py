import os
import time
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from std_msgs.msg import Int32


# 경고 활성화 (0: beep + fullscreen 창)
# ros2 topic pub -1 /robot1/dispatch_command std_msgs/Int32 "data: 0"

# 경고 비활성화 (1: beep 중지 + 창 닫기)
# ros2 topic pub -1 /robot1/dispatch_command std_msgs/Int32 "data: 1"


class TurtlebotControllerNode(Node):
    def __init__(self):
        super().__init__("turtlebot1_controller_node")

        # 상태 변수
        self.alarm_window = None  # tkinter 윈도우 객체
        self.beep_active = False  # beep 상태
        self.beep_thread = None   # beep 쓰레드 객체

        # 비프 퍼블리셔
        self.beep_pub = self.create_publisher(AudioNoteVector, "/robot1/cmd_audio", 10)

        # 명령 수신 구독
        self.create_subscription(
            Int32, "/robot1/dispatch_command", self.dispatch_command_callback, 10
        )

    def dispatch_command_callback(self, msg: Int32):
        cmd = msg.data
        self.get_logger().info(f"명령 수신: {cmd}")

        if cmd == 0:
            self.activate_alert()
        elif cmd == 1:
            self.deactivate_alert()
        else:
            self.get_logger().warn(f"알 수 없는 명령: {cmd}")

    def activate_alert(self):
        if self.beep_active:
            self.get_logger().info("경고 이미 활성화 상태입니다.")
            return

        self.get_logger().warn("경고 활성화: beep + 경고창")
        self.beep_active = True

        # 비프 쓰레드 시작
        self.beep_thread = threading.Thread(target=self.beep_loop, daemon=True)
        self.beep_thread.start()

        # 경고창 쓰레드 실행
        threading.Thread(target=self._run_warning_window, daemon=True).start()

    def deactivate_alert(self):
        if not self.beep_active:
            self.get_logger().info("경고 이미 비활성화 상태입니다.")
            return

        self.get_logger().warn("경고 비활성화: beep 중단 + 창 닫기")
        self.beep_active = False
        self.close_warning_interface()

    def beep_loop(self):
        beep_msg = AudioNoteVector()
        beep_msg.append = False
        beep_msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
        ]

        while self.beep_active:
            self.get_logger().info("비프음 발송 중...")
            self.beep_pub.publish(beep_msg)
            time.sleep(1.0)

    def _run_warning_window(self):
        self.alarm_window = tk.Tk()
        self.alarm_window.title("경고")
        self.alarm_window.attributes("-fullscreen", True)
        self.alarm_window.configure(bg="white")

        label = tk.Label(
            self.alarm_window,
            text="차량 통제 중\n⬅️ ⬅️ ⬅️   ➡️ ➡️ ➡️",
            font=("Arial", 80),
            fg="red",
            bg="white",
        )
        label.pack(expand=True)

        self.get_logger().info("경고창 실행됨")
        self.alarm_window.mainloop()

    def close_warning_interface(self):
        if self.alarm_window:
            try:
                self.get_logger().info("경고창 닫는 중...")
                self.alarm_window.after(0, self.alarm_window.destroy)
                self.alarm_window = None
            except Exception as e:
                self.get_logger().warn(f"경고창 닫기 실패: {e}")


def main(args=None):
    rclpy.init(args=args)

    control_node = TurtlebotControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(control_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
