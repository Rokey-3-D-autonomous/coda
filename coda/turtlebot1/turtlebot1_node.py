import os
import time
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from cv_bridge import CvBridge
import cv2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler


# ===================== [1] 카메라 퍼블리셔 노드 =====================
# 카메라 프레임을 ROS 토픽으로 퍼블리시하는 노드
class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.bridge = CvBridge()

        # Server1이 YOLO 추론에 사용할 카메라 이미지 송신 토픽
        self.pub = self.create_publisher(Image, '/robot1/camera/image_raw', 10)
        
        # 10Hz 주기로 카메라 이미지 publish
        self.timer = self.create_timer(1.0 / 10, self.publish_image)  # 10Hz

        # OpenCV로 카메라 접근
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다.')

    def publish_image(self):
        # 카메라 프레임을 읽고 ROS 메시지로 변환해 publish
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

    def destroy_node(self):
        # 노드 종료 시 카메라 해제
        self.cap.release()
        super().destroy_node()

# ===================== [2] 터틀봇 컨트롤 노드 =====================
# 순찰, 사고 출동, 경보, 종료 등 서버의 명령을 받아서 실행하는 노드
class TurtlebotControllerNode(Node):
    def __init__(self):
        super().__init__('turtlebot1_controller_node')
        
        # Nav2 기반 이동을 위한 navigator 생성
        self.navigator = BasicNavigator()

        # 상태 변수
        self.alarm_window = None        # Tkinter 윈도우 객체
        self.beep_active = False        # beep 상태

        # 반복 비프 퍼블리셔
        self.beep_pub = self.create_publisher(AudioNoteVector, '/robot1/cmd_audio', 10)

        # 순찰 지점 수신 (서버에서 계속 발행)
        self.create_subscription(PoseStamped, '/robot1/patrol_waypoints', self.waypoint_callback, 10)

        # 경보/경고 명령 수신
        self.create_subscription(Int32, '/robot1/dispatch_command', self.dispatch_command_callback, 10)

    def waypoint_callback(self, msg: PoseStamped):
        # 순찰 경로 수신 → 리스트에 추가
        self.get_logger().info(f'순찰 위치 수신: {msg.pose.position}')
        
        # 새로운 목표가 오면 이동
        self.navigator.goToPose(msg)

        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb:
                self.get_logger().info(f'이동 중... 남은 거리: {fb.distance_remaining:.2f} m')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('순찰 도착 완료')
        else:
            self.get_logger().warn('순찰 실패')

    def dispatch_command_callback(self, msg: Int32):
        # 서버에서 온 명령 처리
        cmd = msg.data
        self.get_logger().info(f'명령 수신: {cmd}')

        if cmd == 0:
            self.activate_alert()
        elif cmd == 1:
            self.deactivate_alert()
        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')


    def activate_alert(self):
        # 경고 활성화: 반복 비프 + 경고창
        self.get_logger().warn('경고 활성화: beep + 경고창')
        self.beep_active = True
        threading.Thread(target=self.beep_loop, daemon=True).start()
        self.show_warning_interface()

    def deactivate_alert(self):
        # 경고 비활성화: 비프 정지 + 창 닫기
        self.get_logger().warn('경고 비활성화: beep 중단 + 창 닫기')
        self.beep_active = False
        self.close_warning_interface()

    def play_beep(self):
        # 비프음 반복 전송 (0.5~1초 간격)
        beep_msg = AudioNoteVector()
        beep_msg.append = False
        beep_msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
        ]

        while self.beep_active:
            self.beep_pub.publish(beep_msg)
            time.sleep(1.0)

    def show_warning_interface(self):
        def popup():
            self.alarm_window = tk.Tk()
            self.alarm_window.title("경고")
            self.alarm_window.geometry("300x150")
            label = tk.Label(self.alarm_window, text="이상 상황 발생", font=("Arial", 20))
            label.pack(expand=True)
            self.alarm_window.mainloop()

        threading.Thread(target=popup, daemon=True).start()

    def close_warning_interface(self):
        # 경고창 닫기 시도
        try:
            if self.alarm_window:
                self.alarm_window.destroy()
                self.alarm_window = None
        except Exception as e:
            self.get_logger().warn(f'경고창 닫기 실패: {e}')

    def terminate(self):
        self.get_logger().info('종료 명령 수신 → 종료')
        self.navigator.lifecycleShutdown()
        rclpy.shutdown()

# ===================== [3] 메인 실행 =====================
def main(args=None):
    rclpy.init(args=args)
    
    # 카메라 노드, 컨트롤 노드 실행
    camera_node = CameraPublisherNode()
    control_node = TurtlebotControllerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)
    executor.add_node(control_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
