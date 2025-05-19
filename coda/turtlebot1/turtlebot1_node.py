import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler

# ===================== [1] 카메라 퍼블리셔 노드 =====================
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
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub.publish(msg)

    def destroy_node(self):
        # 종료 시 카메라 해제
        self.cap.release()
        super().destroy_node()

# ===================== [2] 터틀봇 컨트롤 노드 =====================
class TurtlebotControllerNode(Node):
    def __init__(self):
        super().__init__('turtlebot1_controller_node')
        
        # Nav2 기반 이동을 위한 navigator 생성
        self.navigator = BasicNavigator()

        # 상태 변수
        self.current_waypoints = []     # 서버로부터 받은 순찰 경로 리스트
        self.current_index = 0          # 현재 진행 중인 waypoint index
        self.is_patrolling = False      # 순찰 중 여부
        self.is_dispatched = False      # 사고 출동 중 여부

        # 명령 구독
        self.create_subscription(String, '/robot1/command', self.command_callback, 10)

        # 순찰 경로 구독
        self.create_subscription(PoseStamped, '/robot1/patrol_waypoints', self.waypoint_callback, 10)

        # 사고 위치 구독
        self.create_subscription(PoseStamped, '/robot1/accident_pose', self.accident_callback, 10)

    def waypoint_callback(self, msg: PoseStamped):
        # 순찰 경로 수신 → 리스트에 추가
        self.get_logger().info(f'순찰 경로 수신: {msg.pose.position}')
        self.current_waypoints.append(msg)

    def command_callback(self, msg: String):
        # 서버로부터 명령 수신
        cmd = msg.data.lower()
        self.get_logger().info(f'명령 수신: {cmd}')
        if cmd == 'start_patrol':
            self.start_patrol()
        elif cmd == 'resume_patrol':
            self.resume_patrol()
        elif cmd == 'play_alarm':
            self.play_alarm()
        elif cmd == 'terminate':
            self.terminate()

    def accident_callback(self, msg: PoseStamped):
        # 사고 감지 → 사고 위치로 출동
        self.get_logger().warn(f'사고 위치 수신 → 출동 시작')
        self.is_dispatched = True
        
        # 출동 명령
        self.navigator.goToPose(msg)

        while not self.navigator.isTaskComplete():
            fb = self.navigator.getFeedback()
            if fb:
                self.get_logger().info(f'출동 중... 남은 거리: {fb.distance_remaining:.2f} m')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('출동 완료')
        else:
            self.get_logger().error('출동 실패')

    def start_patrol(self):
        # 순찰 시작 명령 수신 시
        if not self.current_waypoints:
            self.get_logger().warn('순찰 경로가 없음')
            return
        self.is_patrolling = True
        self.current_index = 0
        self.patrol_loop()

    def resume_patrol(self):
        # 순찰 재개 명령 수신 시
        if self.current_index < len(self.current_waypoints):
            self.get_logger().info('순찰 재개')
            self.is_patrolling = True
            self.patrol_loop()
        else:
            self.get_logger().info('순찰 이미 완료됨')

    def patrol_loop(self):
        # 순찰 로직 수행
        while self.is_patrolling and self.current_index < len(self.current_waypoints):
            pose = self.current_waypoints[self.current_index]
            self.get_logger().info(f'🚶 순찰 {self.current_index + 1}/{len(self.current_waypoints)} 이동 중')
            
            # 이동 시작
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                fb = self.navigator.getFeedback()
                if fb:
                    self.get_logger().info(f'📡 남은 거리: {fb.distance_remaining:.2f} m')

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('도착 완료')
            else:
                self.get_logger().warn('이동 실패')

            self.current_index += 1

        self.get_logger().info('순찰 완료')
        self.is_patrolling = False

    def play_alarm(self):
        # 서버가 알람 출력 요청 시
        self.get_logger().warn('경보 울림 (사운드/시각적 처리 필요)')

    def terminate(self):
        # 종료 명령 수신 시
        self.get_logger().info('종료 명령 수신 → 도킹 및 종료')
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
