import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.action import ActionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo

# nav
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

# 사용자 정의 인터페이스
from coda_interfaces.action import Patrol
from coda_interfaces.srv import VehicleControl
from coda_interfaces.msg import DispatchCommand

import threading, os, sys, time

# ========================================= 순찰 노드 =========================================
# Server1으로부터 Action Goal을 수신해 순찰 수행
class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Server1의 ActionClient와 통신하기 위한 ActionServer 생성
        self._action_server = ActionServer(
            self,
            Patrol,                         # 사용자 정의 Patrol 액션 타입
            '/turtlebot1/patrol',           # Server1이 이 토픽으로 goal을 보냄
            self.execute_patrol_callback    # goal 수신 시 실행할 콜백 함수
        )

        # 상태 변수
        self.patrol_paused = False          # 사고 발생 시 순찰 일시 정지 여부
        self.patrol_completed = False       # 전체 순찰 완료 여부
        self.current_goal_index = 0         # 현재 순찰 중인 waypoint 인덱스

        # 순찰에 사용할 waypoint 경로 로드
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        # waypoint 좌표 업데이트 필요
        return [
            [2.5, 0.0, 0.0],    # 0. 출발점
            [2.5, 2.5, 180.0],   # 1. 오른쪽 상단
            [0.0, 2.5, 270.0],   # 2. 왼쪽 상단
            [0.0, 0.0, 0.0],     # 3. 왼쪽 하단
            [2.5, 0.0, 90.0],    # 4. 오른쪽 하단
        ]

    def navigate_to(self, pose):
        # 실제 로봇을 이동시키는 부분
        self.get_logger().info(f'\U0001f6f5 {pose}로 이동 중...')
        time.sleep(2)

    async def execute_patrol_callback(self, goal_handle):
        # Action goal 수신 시 호출
        self.get_logger().info('순찰 명령 수신')

        feedback_msg = Patrol.Feedback()    # 피드백 메시지 객체 생성
        self.patrol_completed = False       # 순찰 완료 초기화
        self.patrol_paused = False          # 순찰 정지 상태 해제
        self.current_goal_index = 0

        # 각 waypoint를 순차적으로 이동
        for i, pose in enumerate(self.waypoints):
            self.current_goal_index = i
            
            # 차량 통제로 인한 일시 정지 대응
            while self.patrol_paused:
                self.get_logger().info('순찰 일시 정지됨. 재개 대기 중...')
                time.sleep(1)

            self.navigate_to(pose)

            # 이동 결과 피드백 전송
            feedback_msg.status = f'{i+1}/{len(self.waypoints)} 도착'
            goal_handle.publish_feedback(feedback_msg)

        # 모든 경로 완료. ActionResult 반환
        self.patrol_completed = True
        goal_handle.succeed()

        result = Patrol.Result()
        result.success = True
        self.get_logger().info('순찰 완료')
        return result

# ========================================= 차량 통제 노드 =========================================
# 사고 발생 시 Server1이 호출하는 VehicleControl 서비스 서버. 순찰을 중단시키고 TB2 출동 명령 발송
class VehicleControlReceiverNode(Node):
    def __init__(self, patrol_node: PatrolNode, dispatch_publisher):
        super().__init__('vehicle_control_node')

        # 서비스 서버 생성: Server1이 '/turtlebot1/vehicle_control'로 요청 보냄
        self._srv = self.create_service(
            VehicleControl,
            '/turtlebot1/vehicle_control',      # 서비스 이름
            self.vehicle_control_callback       # 요청 수신 시 실행할 함수
        )
        self.patrol_node = patrol_node          # 순찰 노드 참조 (상태 제어 목적)
        self.dispatch_pub = dispatch_publisher  # TB2로 사고 출동 명령 퍼블리시

    def vehicle_control_callback(self, request, response):
        # 서비스 요청 수신 시 순찰 중단, 경보 발생, 출동 명령 전송
        self.get_logger().warn('차량 통제 명령 수신 → 순찰 일시 정지')  
        self.patrol_node.patrol_paused = True   # 순찰 일시 정지
        
        self.trigger_alert_display()            # 디스플레이 경고
        self.trigger_alert_sound()              # 경고음 출력

        # 출동 명령 메시지 생성 및 퍼블리시
        dispatch_msg = DispatchCommand()
        dispatch_msg.pose = request.pose        # Server1이 보낸 사고 위치 전달
        self.dispatch_pub.publish(dispatch_msg)

        response.success = True                 # 서비스 응답 반환
        return response

    def trigger_alert_display(self):
        # 경고 메시지 출력 (GUI 또는 디스플레이 상에)
        self.get_logger().warn('경고 메시지 표시')

    def trigger_alert_sound(self):
        # beep 코드 작성
        self.get_logger().warn('경고음 출력')

# ========================================= 복귀 수신 노드 =========================================
class RecoveryReceiverNode(Node):
    def __init__(self, patrol_node: PatrolNode):
        super().__init__('recovery_receiver_node')
        self.create_subscription(String, '/turtlebot1/recovery_command', self.recovery_callback, 10)
        self.patrol_node = patrol_node

    def recovery_callback(self, msg):
        self.get_logger().info('복귀 명령 수신')
        if self.patrol_node.patrol_completed:
            self.get_logger().info('복귀 완료 → Server1에 종료 메시지 송신 필요')
        else:
            self.get_logger().info('순찰 재개')
            self.patrol_node.patrol_paused = False

# ========================================= 메인 서버 노드 =========================================
class Turtlebot1Server():
    def __init__(self):
        self.patrol_node = PatrolNode()
        self.dispatch_pub = self.patrol_node.create_publisher(DispatchCommand, '/turtlebot2/dispatch_command', 10)
        self.vehicle_control_node = VehicleControlReceiverNode(self.patrol_node, self.dispatch_pub)
        self.recovery_node = RecoveryReceiverNode(self.patrol_node)

# ========================================= 실행 메인 함수 =========================================
def main(args=None):
    rclpy.init(args=args)
    server = Turtlebot1Server()
    executor = MultiThreadedExecutor()
    executor.add_node(server.patrol_node)
    executor.add_node(server.vehicle_control_node)
    executor.add_node(server.recovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.patrol_node.destroy_node()
        server.vehicle_control_node.destroy_node()
        server.recovery_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
