#!/usr/bin/env python3

import numpy as np

if not hasattr(np, "float"):
    np.float = float

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32  # 예시용 토픽 타입
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Point
from tf_transformations import quaternion_from_euler

# ======================
# 초기 설정 (파일 안에서 직접 정의)
# ======================
INITIAL_POSE_POSITION = [-0.01, -0.01]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH
# INITIAL_POSE = (0.00, 0.00, TurtleBot4Directions.NORTH)
"""
- 1 
x: -0.60
y: 0.26
- 2
x: -0.60
y: 0.26
- 3
x: -0.60
y: 0.26
- 4
x: -0.70
y: 5.33
- 5
x: -1.92
y: 5.53
- 6
x: -1.76
y: 3.77
- 7
x: -1.67
y: 1.54
- 8
x: -1.61
y: -0.38
- 사고지점
x: -1.69
y: 2.10
"""
GOAL_POSES = [
    # patrol route
    ([-1.92, 5.33], TurtleBot4Directions.NORTH),  # 5
    ([-1.76, 3.77], TurtleBot4Directions.NORTH),  # 6
    ([-1.67, 1.54], TurtleBot4Directions.NORTH),  # 7
    ([-1.61, -0.38], TurtleBot4Directions.NORTH),  # 8
    # last pose is in front of docking station
    # ([...]) -1
]
ACCIDENT_POSE = [0.01, -0.01]

# robot namespace
TB0_NAMESPACE = "/robot0"  # photo
# TB1_NAMESPACE = "/robot1"  # patrol

INIT_LOADING_TIME = 3.0
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
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name="navigator_node")
        self.get_logger().info("initialize navigator")

        self.current_goal = 0  # 현재 목표 위치
        self.goal_total = len(GOAL_POSES)  # 총 목표 개수
        self.pending_goal = False  # 현재 이동 중인지 여부

        self.setup_navigation()

        # 도착 완료 시 보낼 퍼블리셔, timer 콜백에서 실행될 퍼블리셔
        self.goal_pub = self.create_publisher(Int32, TB0_NAMESPACE + "/goal_result", 10)

        # 목표 지점 명령
        self.subscription = self.create_subscription(
            Int32, TB0_NAMESPACE + "/goal_position", self.move_to_goal, 10
        )
        self.subscription2 = self.create_subscription(
            Point, TB0_NAMESPACE + "/goal_position2", self.move_to_goal2, 10
        )

    def create_pose(self, pose, navigator) -> PoseStamped:
        x, y = pose[0][0], pose[0][1]
        yaw_deg = pose[1]
        self.get_logger().info(f"x값 {x}")

        """x, y, yaw(도 단위) → PoseStamped 생성"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        yaw_rad = yaw_deg * 3.141592 / 180.0
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def setup_navigation(self):
        if not self.dock_navigator.getDockedStatus():
            self.dock_navigator.info("Docking before initializing pose")
            self.dock_navigator.dock()

        initial_pose = self.create_pose(
            (INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION), self.nav_navigator
        )
        self.nav_navigator.setInitialPose(initial_pose)

        self.get_logger().info("incoming")
        self.nav_navigator.waitUntilNav2Active()
        self.get_logger().info("done waiting for nav active")
        self.dock_navigator.undock()

    def move_to_goal2(self, msg: Point):
        """
        detection 후 tf한 촬영 좌표로 이동
        """
        x, y, z = msg.x, msg.y, msg.z
        goal_pose = self.create_pose(([x, y], 0), self.nav_navigator)

        self.get_logger().info(f"📍 목표 {goal_pose.x}, {goal_pose.y}")

        self.nav_navigator.goToPose(goal_pose)

        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                self.nav_navigator.get_logger().info(f"남은 거리: {remaining:2f} m")

        result = self.nav_navigator.getResult()

        result_topic = Int32()
        result_topic.data = (
            1 if result == TaskResult.SUCCEEDED else -2
        )  # 제대로 도착했으면 goal위치, 아니면 -1
        self.goal_pub.publish(result_topic)  # 결과 퍼블리시

        if result == TaskResult.SUCCEEDED:
            self.pending_goal = False  # 목표 이동 완료
            self.nav_navigator.get_logger().info(
                f"🏁 목표 {self.current_goal} 도달 성공"
            )
            # self.go_into_dock()
        elif result == TaskResult.CANCELED:
            self.nav_navigator.get_logger().warn("이동이 취소되었습니다.")
        elif result == TaskResult.FAILED:
            self.nav_navigator.get_logger().error(f"❌ 목표 {self.current_goal} 실패")
        else:
            self.nav_navigator.get_logger().warn("알 수 없는 오류 발생")

    def move_to_goal(self, msg):
        if msg.data == -1:
            # self.get_logger().warn(f"⚠️ 잘못된 목표 인덱스: {msg.data}")
            self.get_logger().info(f"DOCKING TB0")
            # docking
            self.go_into_dock()
            result_topic = Int32()
            result_topic.data = -1
            self.goal_pub.publish(result_topic)  # 결과 퍼블리시
            return

        if msg.data < 0 or msg.data >= self.goal_total:
            # self.get_logger().warn(f"⚠️ 잘못된 목표 인덱스: {msg.data}")
            self.get_logger().info(
                f"go to last position in front of dock station: {msg.data}"
            )
            return

        self.pending_goal = True  # 목표 이동 중
        self.current_goal = msg.data  # 현재 목표 위치

        position, direction = GOAL_POSES[self.current_goal]
        goal_pose = self.create_pose((position, direction), self.nav_navigator)

        self.get_logger().info(f"📍 목표 {msg.data}")

        self.nav_navigator.goToPose(goal_pose)

        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                self.nav_navigator.get_logger().info(f"남은 거리: {remaining:2f} m")

        result = self.nav_navigator.getResult()

        result_topic = Int32()
        result_topic.data = (
            self.current_goal if result == TaskResult.SUCCEEDED else -1
        )  # 제대로 도착했으면 goal위치, 아니면 -1
        self.goal_pub.publish(result_topic)  # 결과 퍼블리시

        if result == TaskResult.SUCCEEDED:

            self.pending_goal = False  # 목표 이동 완료
            self.nav_navigator.get_logger().info(
                f"🏁 목표 {self.current_goal} 도달 성공"
            )
            # self.go_into_dock()
        elif result == TaskResult.CANCELED:
            self.nav_navigator.get_logger().warn("이동이 취소되었습니다.")
        elif result == TaskResult.FAILED:
            self.nav_navigator.get_logger().error(f"❌ 목표 {self.current_goal} 실패")
        else:
            self.nav_navigator.get_logger().warn("알 수 없는 오류 발생")

    # 컨트롤 서버에서 토픽 전송 시 호출되는 콜백
    # 도킹 시작
    def go_into_dock(self):
        self.dock_navigator.get_logger().info("✅ 모든 목표 도달 완료. 도킹 시작")
        self.dock_navigator.dock()
        self.dock_navigator.get_logger().info("✅ 도킹 요청 완료")


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
