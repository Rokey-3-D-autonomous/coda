import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor

# 사용자 정의 인터페이스 (수정 필요)
from coda_interfaces.action import Patrol
from coda_interfaces.srv import VehicleControl
from coda_interfaces.msg import DispatchCommand

import time

# ========================================= 순찰 노드 =========================================
class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self._action_server = ActionServer(
            self,
            Patrol,
            '/turtlebot1/patrol',
            self.execute_patrol_callback
        )

        self.patrol_paused = False
        self.patrol_completed = False
        self.current_goal_index = 0
        self.waypoints = self.load_waypoints()

    def load_waypoints(self):
        return [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 90.0]
        ]

    def navigate_to(self, pose):
        self.get_logger().info(f'\U0001f6f5 {pose}로 이동 중...')
        time.sleep(2)

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

# ========================================= 차량 통제 노드 =========================================
class VehicleControlReceiverNode(Node):
    def __init__(self, patrol_node: PatrolNode, dispatch_publisher):
        super().__init__('vehicle_control_node')
        self._srv = self.create_service(
            VehicleControl,
            '/turtlebot1/vehicle_control',
            self.vehicle_control_callback
        )
        self.patrol_node = patrol_node
        self.dispatch_pub = dispatch_publisher

    def vehicle_control_callback(self, request, response):
        self.get_logger().warn('차량 통제 명령 수신 → 순찰 일시 정지')
        self.patrol_node.patrol_paused = True
        self.trigger_alert_display()
        self.trigger_alert_sound()

        dispatch_msg = DispatchCommand()
        dispatch_msg.pose = request.pose
        self.dispatch_pub.publish(dispatch_msg)

        response.success = True
        return response

    def trigger_alert_display(self):
        self.get_logger().warn('경고 메시지 표시')

    def trigger_alert_sound(self):
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
