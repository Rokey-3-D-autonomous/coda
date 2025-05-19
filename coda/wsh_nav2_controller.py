#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # 예시용 토픽 타입
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [0.00, 0.00]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

GOAL_POSES = [
    ([-0.87, -1.21], TurtleBot4Directions.NORTH),
    ([-0.87, -1.21], TurtleBot4Directions.NORTH),
    ([-0.87, -1.21], TurtleBot4Directions.NORTH),
    ([-0.87, -1.21], TurtleBot4Directions.NORTH),
]
# ======================
"""
def setup_navigation(self): 네비게이션 셋업
def move_to_goal(self, msg): 목표 이동
def go_into_dock(self): 도킹 시작, 함수 구현완료, 현재는 노드 내에 실행되는 부분은 없음
"""

"""
TODO
1. locailiation 실행
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=$HOME/rokey_ws/maps/first_map.yaml
2. rviz 실행
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot4
3. 이 노드 실행
4. 목적지 도착 하는지 토픽 확인 = ros2 topic echo /robot1/goal_result
5. 실행
# 실행 예시 터미널에서
# ros2 topic pub /robot1/move_position std_msgs/Int32 "data: 1"
"""


# ======================
class NavController(Node):
    def __init__(self):
        super().__init__("nav_controller_node")
        self.navigator = TurtleBot4Navigator()

        self.current_goal = 0  # 현재 목표 위치
        self.goal_total = len(GOAL_POSES)  # 총 목표 개수
        self.pending_goal = False  # 현재 이동 중인지 여부

        self.setup_navigation()

        # 도착 완료 시 보낼 퍼블리셔, timer 콜백에서 실행될 퍼블리셔
        self.goal_pub = self.create_publisher(Int32, "/robot1/goal_result", 10)

        # 목표 지점 명령
        self.subscription = self.create_subscription(
            Int32, "/robot1/move_position", self.move_callback, 10
        )

    def setup_navigation(self):
        if not self.navigator.getDockedStatus():
            self.navigator.info("Docking before initializing pose")
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped(
            INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION
        )
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

    def move_to_goal(self, msg):
        if msg.data < 0 or msg.data >= self.goal_total:
            self.get_logger().warn(f"⚠️ 잘못된 목표 인덱스: {msg.data}")
            return

        self.pending_goal = True  # 목표 이동 중
        self.current_goal = msg.data  # 현재 목표 위치

        position, direction = GOAL_POSES[self.current_goal]
        goal_pose = self.navigator.getPoseStamped(position, direction)

        self.get_logger().info(f"📍 목표 {msg.data}")

        result = self.navigator.goToPose(goal_pose)

        result_topic = Int32()
        result_topic.data = (
            self.current_goal if result.succeeded else -1
        )  # 제대로 도착했으면 goal위치, 아니면 -1
        self.goal_pub.publish(result_topic)  # 결과 퍼블리시

        if result.succeeded:
            self.pending_goal = False  # 목표 이동 완료
            self.get_logger().info(f"🏁 목표 {self.current_goal} 도달 성공")
        else:
            self.get_logger().error(f"❌ 목표 {self.current_goal} 실패")

    # 컨트롤 서버에서 토픽 전송 시 호출되는 콜백
    # 도킹 시작
    def go_into_dock(self):
        self.get_logger().info("✅ 모든 목표 도달 완료. 도킹 시작")
        self.navigator.dock()


def main():
    rclpy.init()
    node = NavController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# 실행 예시
# ros2 topic pub /robot1/move_position std_msgs/Int32 "data: 1"
