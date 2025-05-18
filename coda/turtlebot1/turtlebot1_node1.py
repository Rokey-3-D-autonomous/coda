# ROS2 기본 패키지 import
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

# 메시지 타입 import
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# 사용자 정의 인터페이스 import
from coda_interfaces.action import Patrol           # Server1이 보내는 순찰 Action
from coda_interfaces.srv import VehicleControl      # 사고 발생 시 호출하는 서비스
from coda_interfaces.msg import DispatchCommand     # Turtlebot2 출동 명령용 메시지

import time     # 시뮬레이션 대기 시간 처리용

# ========================================= 순찰 노드 =========================================
# Server1으로부터 Action Goal을 받아 지정된 waypoint를 따라 순찰을 수행
class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Patrol.action을 처리하는 Action Server 생성
        self._action_server = ActionServer(
            self,
            Patrol,                         # 액션 타입
            '/turtlebot1/patrol',           # Goal을 받을 토픽
            self.execute_patrol_callback    # 실행 콜백 함수
        )

        # 상태 변수 초기화
        self.patrol_paused = False          # 사고 발생 시 순찰 일시 정지
        self.patrol_completed = False       # 순찰 완료 여부
        self.current_goal_index = 0         # 순찰 중인 waypoint index
        self.waypoints = []                 # Server1이 넘겨주는 waypoint 리스트

    def navigate_to(self, pose: PoseStamped):
        # 실제 이동 로직
        self.get_logger().info(f'이동 중: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        time.sleep(2)       # 추후 Nav2로 대체

    async def execute_patrol_callback(self, goal_handle):
        # Server1이 순찰 Goal을 보내면 실행되는 메소드
        self.get_logger().info('순찰 명령 수신')
        
        self.waypoints = goal_handle.request.waypoints      # Server1이 보낸 좌표 저장
        
        # 상태 초기화
        feedback_msg = Patrol.Feedback()
        self.patrol_paused = False
        self.patrol_completed = False

        # 각 waypoint를 순차적으로 수행
        for i, pose in enumerate(self.waypoints):
            self.current_goal_index = i

            # 사고 발생으로 순찰 중단된 경우 대기
            while self.patrol_paused:
                self.get_logger().info('순찰 일시 정지 중...')  # 차량 통제 중 대기
                time.sleep(1)

            self.navigate_to(pose)      # 실제 pose로 이동
            
            # Server1에 현재 상태 피드백 전송
            feedback_msg.status = f'{i+1}/{len(self.waypoints)} 도착'
            goal_handle.publish_feedback(feedback_msg)

        # 전체 경로 완료 처리
        self.patrol_completed = True
        goal_handle.succeed()

        result = Patrol.Result()
        result.success = True
        self.get_logger().info('순찰 완료')
        return result

# ========================================= 차량 통제 서비스 노드 =========================================
# Server1이 사고를 감지했을 때 호출하는 서비스 처리 노드
# 순찰을 중지하고 Turtlebot2에게 사고 출동 요청을 발행
class VehicleControlNode(Node):
    def __init__(self, patrol_node: PatrolNode, dispatch_pub):
        super().__init__('vehicle_control_node')
        self.patrol_node = patrol_node
        self.dispatch_pub = dispatch_pub        # Turtlebot2에게 메시지 보낼 퍼블리셔

        self.create_service(
            VehicleControl,
            '/turtlebot1/vehicle_control',      # Server1이 호출하는 서비스명
            self.handle_vehicle_control
        )

    def handle_vehicle_control(self, request, response):
        # 사고 발생 → 순찰 정지 → Turtlebot2 출동 명령 전송
        self.get_logger().warn('차량 통제 명령 수신 → 순찰 정지 및 출동 요청')
        self.patrol_node.patrol_paused = True

        # Turtlebot2 출동 명령 생성 및 퍼블리셔
        dispatch_msg = DispatchCommand()
        dispatch_msg.pose = request.pose
        self.dispatch_pub.publish(dispatch_msg)

        # 경고 처리 (디스플레이, 사운드)
        self.get_logger().warn('경고 메시지 표시')
        self.get_logger().warn('경고음 울림')

        response.success = True
        return response

# ========================================= 복귀 보고 수신 노드 =========================================
# Turtlebot2가 복귀를 마치고 Turtlebot1에 복귀 완료 보고를 보내는 수신 처리 노드
# 순찰이 완료된 경우 → 종료
# 순찰이 미완료된 경우 → 순찰 재개
class RecoveryReportNode(Node):
    def __init__(self, patrol_node: PatrolNode):
        super().__init__('recovery_report_node')
        self.patrol_node = patrol_node

        self.create_subscription(
            String,
            '/turtlebot1/recovery_report',      # Turtlebot2가 보내는 토픽
            self.handle_recovery,
            10
        )

    def handle_recovery(self, msg: String):
        self.get_logger().info('복귀 보고 수신')

        if self.patrol_node.patrol_completed:
            self.get_logger().info('순찰 이미 완료 → 종료 대기')
        else:
            self.get_logger().info('순찰 재개')
            self.patrol_node.patrol_paused = False

# ========================================= 노드 통합 서버 =========================================
# 각 노드를 통합하고 실행을 관리하는 상위 클래스
class Turtlebot1Server:
    def __init__(self):
        self.patrol_node = PatrolNode()

        # Turtlebot2에게 사고 출동 명령을 보내는 퍼블리셔 생성
        self.dispatch_pub = self.patrol_node.create_publisher(
            DispatchCommand,
            '/turtlebot2/dispatch_command',
            10
        )

        # 차량 통제 수신 노드 + 복귀 보고 수신 노드 초기화
        self.vehicle_control_node = VehicleControlNode(self.patrol_node, self.dispatch_pub)
        self.recovery_node = RecoveryReportNode(self.patrol_node)

# ========================================= 메인 실행 =========================================
def main(args=None):
    rclpy.init(args=args)
    server = Turtlebot1Server()

    # MultiThreadedExecutor 사용: 노드들을 병렬로 실행
    executor = MultiThreadedExecutor()
    executor.add_node(server.patrol_node)
    executor.add_node(server.vehicle_control_node)
    executor.add_node(server.recovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 리소스 정리
        server.patrol_node.destroy_node()
        server.vehicle_control_node.destroy_node()
        server.recovery_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
