import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# 가정: 사용자 정의 인터페이스 (사용 환경에 따라 수정 필요)
from coda_interfaces.action import Patrol
from coda_interfaces.srv import VehicleControl
from coda_interfaces.msg import DispatchCommand

import time

class Turtlebot1Node(Node):
    def __init__(self):
        super().__init__('turtlebot1_node')

        # 순찰 액션 서버 생성
        self._action_server = ActionServer(
            self,
            Patrol,
            '/turtlebot1/patrol',
            self.execute_patrol_callback
        )

        # 차량 통제 서비스 수신 서버
        self._vehicle_control_srv = self.create_service(
            VehicleControl,
            '/turtlebot1/vehicle_control',
            self.vehicle_control_callback
        )

        # 출동 명령 퍼블리셔 (Turtlebot2에게 전송)
        self._dispatch_pub = self.create_publisher(DispatchCommand, '/turtlebot2/dispatch_command', 10)

        # 복귀 명령 수신 (Turtlebot2 → Turtlebot1)
        self._recovery_sub = self.create_subscription(
            String,
            '/turtlebot1/recovery_command',
            self.recovery_callback,
            10
        )

        # 내부 상태
        self.patrol_paused = False
        self.patrol_completed = False
        self.current_goal_index = 0
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        # 실제 좌표로 교체 필요
        return [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 90.0]
        ]

    def navigate_to(self, pose):
        # 실제 네비게이션 명령으로 대체 필요
        self.get_logger().info(f'\U0001f6f5 {pose}로 이동 중...')
        time.sleep(2)  # 더미 대기

    async def execute_patrol_callback(self, goal_handle):
        self.get_logger().info('순찰 명령 수신')

        feedback_msg = Patrol.Feedback()

        self.patrol_completed = False
        self.patrol_paused = False
        self.current_goal_index = 0

        for i, pose in enumerate(self.waypoints):
            self.current_goal_index = i
            while self.patrol_paused:
                self.get_logger().info('순찰 일시 정지됨. 재개 대기 중...')
                time.sleep(1)

            self.navigate_to(pose)

            feedback_msg.status = f'{i+1}/{len(self.waypoints)} 도착'
            goal_handle.publish_feedback(feedback_msg)

        self.patrol_completed = True
        goal_handle.succeed()

        result = Patrol.Result()
        result.success = True
        self.get_logger().info('순찰 완료')
        return result

    def vehicle_control_callback(self, request, response):
        self.get_logger().warn('차량 통제 명령 수신 → 순찰 일시 정지')
        self.patrol_paused = True

        self.trigger_alert_display()
        self.trigger_alert_sound()

        dispatch_msg = DispatchCommand()
        dispatch_msg.pose = request.pose  # 사고 위치 전송
        self._dispatch_pub.publish(dispatch_msg)

        response.success = True
        return response

    def recovery_callback(self, msg):
        self.get_logger().info('복귀 명령 수신')
        if self.patrol_completed:
            self.get_logger().info('복귀 완료 → Server1에 종료 메시지 송신 필요')
            # 실제로는 Action result를 외부에 보내야 함 (goal_handle 참조 필요)
        else:
            self.get_logger().info('순찰 재개')
            self.patrol_paused = False

    def trigger_alert_display(self):
        self.get_logger().warn('경고 메시지 표시')

    def trigger_alert_sound(self):
        self.get_logger().warn('경고음 출력')

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()