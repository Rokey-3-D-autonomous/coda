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


class NavController(Node):
    def __init__(self):
        super().__init__("nav_controller_node")
        self.navigator = TurtleBot4Navigator()

        self.goal_total = len(GOAL_POSES)

        self.setup_navigation()

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

    def move_callback(self, msg):
        if msg.data < 0 or msg.data >= self.goal_total:
            self.get_logger().warn(f"⚠️ 잘못된 목표 인덱스: {msg.data}")
            return
        position, direction = GOAL_POSES[msg.data]
        goal_pose = self.navigator.getPoseStamped(position, direction)

        self.navigator.startToPose(goal_pose)
        result = self.navigator.goToPose()

        self.get_logger().info(f"📍 목표 {msg.data}")

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
